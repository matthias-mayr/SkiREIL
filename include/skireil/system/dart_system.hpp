//|
//|    Copyright Inria July 2017
//|    This project has received funding from the European Research Council (ERC) under
//|    the European Union's Horizon 2020 research and innovation programme (grant
//|    agreement No 637972) - see http://www.resibots.eu
//|
//|    Contributor(s):
//|      - Matthias Mayr (matthias.mayr@cs.lth.se)
//|      - Konstantinos Chatzilygeroudis (konstantinos.chatzilygeroudis@inria.fr)
//|      - Rituraj Kaushik (rituraj.kaushik@inria.fr)
//|      - Roberto Rama (bertoski@gmail.com)
//|
//|
//|    This software is governed by the CeCILL-C license under French law and
//|    abiding by the rules of distribution of free software.  You can  use,
//|    modify and/ or redistribute the software under the terms of the CeCILL-C
//|    license as circulated by CEA, CNRS and INRIA at the following URL
//|    "http://www.cecill.info".
//|
//|    As a counterpart to the access to the source code and  rights to copy,
//|    modify and redistribute granted by the license, users are provided only
//|    with a limited warranty  and the software's author,  the holder of the
//|    economic rights,  and the successive licensors  have only  limited
//|    liability.
//|
//|    In this respect, the user's attention is drawn to the risks associated
//|    with loading,  using,  modifying and/or developing or reproducing the
//|    software by the user in light of its specific status of free software,
//|    that may mean  that it is complicated to manipulate,  and  that  also
//|    therefore means  that it is reserved for developers  and  experienced
//|    professionals having in-depth computer knowledge. Users are therefore
//|    encouraged to load and test the software's suitability as regards their
//|    requirements in conditions enabling the security of their systems and/or
//|    data to be ensured and,  more generally, to use and operate it in the
//|    same conditions as regards security.
//|
//|    The fact that you are presently reading this means that you have had
//|    knowledge of the CeCILL-C license and that you accept its terms.
//|
#ifndef SKIREIL_SYSTEM_DART_SYSTEM_HPP
#define SKIREIL_SYSTEM_DART_SYSTEM_HPP

#include <functional>

#include <robot_dart/control/robot_control.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/utils.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/collision/CollisionResult.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include <skireil/parameters.hpp>
#include <skireil/system/system.hpp>
#include <skireil/utils/utils.hpp>
#include <skireil/experiments/learning_skills_execution.hpp>
#include <skireil/reward/reward_result.hpp>

#include <glog/logging.h>

using ParamVec = std::vector<std::shared_ptr<skireil::parameters::ParamBase>>;

namespace skireil {
    namespace system {
        template <typename Params>
        void make_peg_and_ph_collide(robot_dart::RobotDARTSimu& simu) {
            bool polyhedron{false};
            for (const auto& o : Params::skireil::state_objects()) {
                if (o == "polyhedron_link") {
                    polyhedron = true;
                    break;
                }
            }
            if (polyhedron) {
                // Enable collision between the peg and the polygedron
                // Find robot
                const auto &robots = simu.robots();
                std::shared_ptr<robot_dart::Robot> robot;
                for (const auto& r : robots) {
                    if (r->name() == "arm") {
                        robot = r;
                        break;
                    }
                }
                CHECK_NOTNULL(robot);
                int robot_index = simu.robot_index(robot);
                robot->set_self_collision(true);
                simu.set_collision_mask(robot_index, 1 << 0);
                simu.set_collision_mask(robot_index, "polyhedron_link", 1 << 1);
            }
        }


        template <typename Params, typename PolicyController, typename RolloutInfo>
        struct DARTSystem : public System<Params, DARTSystem<Params, PolicyController, RolloutInfo>, RolloutInfo> {
            using robot_simu_t = robot_dart::RobotDARTSimu;

            DARTSystem() {
                if (Params::skireil::learn_real_system()) {
                    _execution = std::make_shared<learning_skills_execution<PolicyController, Params>>();
                }
            }

            template <typename Policy, typename Reward>
            std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>> execute_with_real_system(const Policy& policy, Reward& world, double T, reward::RewardResultMap& R) {
                    // Consists of: init_transformed_state, u, diff_state
                    std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>> res;

                    // Get the information of the rollout
                    RolloutInfo rollout_info = this->get_rollout_info();
                    // setup robot
                    std::shared_ptr<robot_dart::Robot> robot_model = this->get_robot();

                    // setup the controller
                    auto controller = std::make_shared<PolicyController>();
                    controller->configure(policy.params());
                    controller->set_robot_in_policy(robot_model);
                    controller->set_controller_real_params();
                    controller->set_goal(this->_rollout_info.goal);
                    robot_model->add_controller(controller);
                    bool not_done{true};
                    
                    while(not_done) {
                        controller->configure(policy.params());
                        skireil::execution::reset_to_start_pos();
                        // Reset recroded states?
                        not_done = !_execution->run(controller, robot_model, false);
                    }

                    std::vector<Eigen::VectorXd> states = controller->get_states();
                    std::vector<Eigen::VectorXd> noiseless_states = controller->get_noiseless_states();
                    std::vector<Eigen::VectorXd> commands = controller->get_commands();
                    // if (display)  {
                        // this->_last_states = states;
                        // this->_last_dense_states = controller->get_dense_states();
                        // this->_last_commands = commands;
                        // this->_last_dense_commands = controller->get_dense_commands();
                        // this->_last_dense_ee_pos = controller->get_dense_ee_pos();
                        // this->_last_dense_ee_rot = controller->get_dense_ee_rot();
                    // }
                    rollout_info.finish_time = controller->finish_time;
                    // Order results and calculate reward
                    std::vector<Eigen::VectorXd> transformed_state;
                    transformed_state.resize(states.size());
                    for (size_t j = 0; j < states.size() - 1; j++) {
                        transformed_state.at(j) = this->transform_state(states.at(j));
                        res.push_back(std::make_tuple(transformed_state[j], commands[j], states[j + 1] - states[j]));
                    }
                    
                    R = reward::calculate_rewards<Reward, RolloutInfo, Params>(world, rollout_info, noiseless_states, commands);

                    if (!policy.random() && false) {
                        LOG(INFO) << std::boolalpha << "Solution is feasible: " << reward::check_feasibility(R);
                        LOG(INFO) << reward::rewards_to_stream(R).rdbuf();
                        rollout_info.print();
                    }
                    // Hack: This should not be necessary
                    robot_model->clear_controllers();
                    controller.reset();
                    return res;
            }

            template <typename Policy, typename Reward>
            std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>> execute(const Policy& policy, Reward& world, double T, reward::RewardResultMap& R, bool display = true)
            {
                // Make sure that the simulation step is smaller than the sampling/control rate
                assert(Params::dart_system::sim_step() < Params::skireil::dt());

                // Consists of: init_transformed_state, u, diff_state
                std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>> res;

                robot_simu_t simu;
#ifdef GRAPHIC
                simu.set_graphics(std::make_shared<robot_dart::gui::magnum::Graphics>());
                simu.graphics()->set_enable(display);
#endif
                // simulation step different from sampling rate -- we need a stable simulation
                simu.set_timestep(Params::dart_system::sim_step());

                // Get the information of the rollout
                RolloutInfo rollout_info = this->get_rollout_info();

                // setup robot
                std::shared_ptr<robot_dart::Robot> simulated_robot = this->get_robot();

                // setup the controller
                auto controller = std::make_shared<PolicyController>();
                controller->configure(policy.params());
                controller->set_robot_in_policy(this->get_robot());
                controller->set_transform_state(std::bind(&DARTSystem::transform_state, this, std::placeholders::_1));
                controller->set_noise_function(std::bind(&DARTSystem::add_noise, this, std::placeholders::_1));
                controller->set_update_function(std::bind([&](double t) { rollout_info.t = t; }, std::placeholders::_1));
                controller->set_policy_function(std::bind(&DARTSystem::policy_transform, this, std::placeholders::_1, &rollout_info));
                controller->set_goal(this->_rollout_info.goal);

                // Add extra to simu object
                this->add_extra_to_simu(simu, rollout_info);

                // add the controller to the robot
                simulated_robot->add_controller(controller);
                // add the robot to the simulation
                simu.add_robot(simulated_robot);

                // Get initial state from info and add noise
                Eigen::VectorXd init_diff = rollout_info.init_state;
                this->set_robot_state(simulated_robot, init_diff);
                init_diff = this->add_noise(init_diff);

                // Runs the simulation for a maximum of T + sim_step time
                // TODO: Here we would plug in the GP and run this in a loop
                simu.run(T + Params::dart_system::sim_step());

                std::vector<Eigen::VectorXd> states = controller->get_states();
                std::vector<Eigen::VectorXd> noiseless_states = controller->get_noiseless_states();
                std::vector<Eigen::VectorXd> commands = controller->get_commands();
                if (display)  {
                    this->_last_states = states;
                    this->_last_dense_states = controller->get_dense_states();
                    this->_last_commands = commands;
                    this->_last_dense_commands = controller->get_dense_commands();
                    this->_last_dense_ee_pos = controller->get_dense_ee_pos();
                    this->_last_dense_ee_rot = controller->get_dense_ee_rot();
                }
                rollout_info.finish_time = controller->finish_time;

                // Order results and calculate reward
                std::vector<Eigen::VectorXd> transformed_state;
                transformed_state.resize(states.size());
                for (size_t j = 0; j < states.size() - 1; j++) {
                    transformed_state[j] = this->transform_state(states[j]);
                    res.push_back(std::make_tuple(transformed_state[j], commands[j], states[j + 1] - states[j]));
                }

                R = reward::calculate_rewards<Reward, RolloutInfo, Params>(world, rollout_info, noiseless_states, commands);

                if (!policy.random() && display) {
                    LOG(INFO) << std::boolalpha << "Solution is feasible: " << reward::check_feasibility(R);
                    LOG(INFO) << reward::rewards_to_stream(R).rdbuf();
                    rollout_info.print();
                }
                return res;
            }

            template <typename Model>
            std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>> execute_with_sequence(const std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>>& observation, const ParamVec& params, bool display, const Model& model, const std::string& residual_file = "", bool with_variance = false) {
                if (Params::meta_conf::verbose()) {
                    LOG(INFO) << "Rollout of an observed episode in simulation";
                }
                // Make sure that the simulation step is smaller than the sampling/control rate
                assert(Params::dart_system::sim_step() < Params::skireil::dt());

                robot_simu_t simu;
#ifdef GRAPHIC
                simu.set_graphics(std::make_shared<robot_dart::gui::magnum::Graphics>());
                simu.graphics()->set_enable(display);
#endif
                // simulation step different from sampling rate -- we need a stable simulation
                simu.set_timestep(Params::dart_system::sim_step());

                // Get the information of the rollout
                RolloutInfo rollout_info = this->get_rollout_info();

                std::shared_ptr<robot_dart::Robot> simulated_robot = this->get_robot();

                // setup the controller
                auto controller = std::make_shared<PolicyController>();
                controller->configure(params);
                controller->set_robot_in_policy(this->get_robot());
                controller->set_skip_next();
                controller->set_transform_state(std::bind(&DARTSystem::transform_state, this, std::placeholders::_1));
                controller->set_noise_function(std::bind(&DARTSystem::add_noise, this, std::placeholders::_1));
                controller->set_update_function(std::bind([&](double t) { rollout_info.t = t; }, std::placeholders::_1));
                controller->set_policy_function(std::bind(&DARTSystem::policy_transform, this, std::placeholders::_1, &rollout_info));
                controller->set_goal(this->_rollout_info.goal);

                // Add extra to simu object
                this->add_extra_to_simu(simu, rollout_info);

                // add the controller to the robot
                simulated_robot->add_controller(controller);
                // add the robot to the simulation
                simu.add_robot(simulated_robot);

                // // Get initial state from info and add noise
                // Eigen::VectorXd init_diff = rollout_info.init_state;
                // this->set_robot_state(simulated_robot, init_diff);
                // init_diff = this->add_noise(init_diff);

                std::vector<Eigen::VectorXd> states;
                double dt = Params::skireil::dt();
                double T_passed{0};
                for (size_t i = 0; i < observation.size(); i++, T_passed+= dt) {
                    // Setting State and action
                    this->set_robot_state(simulated_robot, std::get<0>(observation[i]));
                    // Alternative 1/2: When using high-level commands:
                    // controller->process_command(std::get<1>(observation[i]), true);
                    // Alternative 2/2: When using joint velocities:
                    controller->set_fixed_action(std::get<1>(observation[i]));
                    if (!residual_file.empty()) {
                        Eigen::VectorXd query_vec(Params::skireil::model_input_dim() + Params::skireil::action_dim());
                        query_vec.head(Params::skireil::model_input_dim()) = std::get<0>(observation[i]);
                        query_vec.tail(Params::skireil::action_dim()) = std::get<1>(observation[i]);
                        Eigen::VectorXd mu;
                        Eigen::VectorXd sigma;
                        std::tie(mu, sigma) = model.predict(query_vec, with_variance);
                        Eigen::VectorXd res_mean{mu};
                        if (with_variance) {
                            sigma = sigma.array().sqrt();
                            for (int i = 0; i < mu.size(); i++) {
                                double s = utils::gaussian_rand(mu(i), sigma(i));
                                mu(i) = std::max(mu(i) - sigma(i),
                                    std::min(s, mu(i) + sigma(i)));
                            }
                        }
                        controller->append_applied_residual(res_mean, res_mean - mu);
                    }

                    simu.run(dt);
                    states.push_back(controller->get_state(simulated_robot));
                    if (Params::meta_conf::verbose()) {
                        LOG(INFO) << "State before: " << std::get<0>(observation[i]).format(OneLine);
                        LOG(INFO) << "Action : " << std::get<1>(observation[i]).format(OneLine);
                        LOG(INFO) << "State after: " << controller->get_state(simulated_robot).format(OneLine);
                    }
                }

                // Consists of: init_state_t, u_t, simulation_state_t+1
                std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>> res;
                for (size_t j = 0; j < observation.size(); j++) {
                    res.push_back(std::make_tuple(std::get<0>(observation[j]), std::get<1>(observation[j]), states[j]));
                }

                if (!residual_file.empty()) {
                    utils::save_traj_to_file(residual_file, controller->get_applied_residual_mean(), controller->get_applied_residual_variance());
                }

                return res;
            }

            template <typename Policy, typename Model, typename Reward>
            std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>> execute_with_model(const Policy& policy, const Model& model, Reward& world, double T, reward::RewardResultMap& R, bool display = true, bool with_variance = true) {
                // Make sure that the simulation step is smaller than the sampling/control rate
                assert(Params::dart_system::sim_step() < Params::skireil::dt());

                // Consists of: init_transformed_state, u, diff_state
                std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>> res;

                robot_simu_t simu;
#ifdef GRAPHIC
                simu.set_graphics(std::make_shared<robot_dart::gui::magnum::Graphics>());
                simu.graphics()->set_enable(display);
#endif
                // simulation step different from sampling rate -- we need a stable simulation
                simu.set_timestep(Params::dart_system::sim_step());

                // Get the information of the rollout
                RolloutInfo rollout_info = this->get_rollout_info();

                // setup robots
                std::shared_ptr<robot_dart::Robot> simulated_robot = this->get_robot();

                // setup the controller
                auto controller = std::make_shared<PolicyController>();
                controller->configure(policy.params());
                controller->set_robot_in_policy(this->get_robot());
                controller->set_transform_state(std::bind(&DARTSystem::transform_state, this, std::placeholders::_1));
                controller->set_noise_function(std::bind(&DARTSystem::add_noise, this, std::placeholders::_1));
                controller->set_update_function(std::bind([&](double t) { rollout_info.t = t; }, std::placeholders::_1));
                controller->set_policy_function(std::bind(&DARTSystem::policy_transform, this, std::placeholders::_1, &rollout_info));
                controller->set_goal(this->_rollout_info.goal);

                // Add extra to simu object
                this->add_extra_to_simu(simu, rollout_info);

                // add the controller to the robot
                simulated_robot->add_controller(controller);
                // add the robot to the simulation
                simu.add_robot(simulated_robot);
                make_peg_and_ph_collide<Params>(simu);

                // Get initial state from info and add noise
                Eigen::VectorXd init_diff = rollout_info.init_state;
                this->set_robot_state(simulated_robot, init_diff);
                init_diff = this->add_noise(init_diff);

                // Runs the simulation for "dt" time, then queries the GP
                // TODO: Optimize for speed?
                if (Params::meta_conf::verbose()) {
                    LOG(INFO) << "Finished setup of the simulation. Running it now." << std::flush;
                }
                double dt = Params::skireil::dt();
                bool deep_collision{false};
                for (double T_passed = 0; T_passed < T; T_passed+= dt) {
                    simu.run(dt);
                    Eigen::VectorXd state = controller->get_state(simulated_robot);
                    Eigen::VectorXd action = controller->get_last_command();
                    if (action.size() < static_cast<int>(Params::skireil::action_dim())) {
                        LOG(WARNING) << "Controller returned an action of size " << action.size() << " which is smaller than the action dim.: " << Params::skireil::action_dim();
                        action = Eigen::VectorXd(Params::skireil::action_dim());
                    }
                    Eigen::VectorXd query_vec(Params::skireil::model_input_dim() + Params::skireil::action_dim());

                    query_vec.head(Params::skireil::model_input_dim()) = state;
                    query_vec.tail(Params::skireil::action_dim()) = action;

                    Eigen::VectorXd mu;
                    Eigen::VectorXd sigma;
                    std::tie(mu, sigma) = model.predict(query_vec, with_variance);
                    Eigen::VectorXd res_mean{mu};
                    if (with_variance) {
                        sigma = sigma.array().sqrt();
                        for (int i = 0; i < mu.size(); i++) {
                            double s = utils::gaussian_rand(mu(i), sigma(i));
                            mu(i) = std::max(mu(i) - sigma(i),
                                std::min(s, mu(i) + sigma(i)));
                        }
                    }
                    controller->append_applied_residual(res_mean, res_mean - mu);
                    //mu.array() /= Params::skireil::gp_scale().array();
                    Eigen::VectorXd final = state + mu;
                    this->set_robot_state(simulated_robot, final);
                    if (check_for_deep_collisions(simu.world()->getLastCollisionResult(), Params::skireil::max_collision_depth())) {
                        deep_collision = true;
                        break;
                    }
                }
                if (Params::meta_conf::verbose()) {
                    LOG(INFO) << "Simulation done. Postprocessing." << std::flush;
                }
                if (deep_collision) {
                    // TODO: Feasibility constraint on deep collisions
                    // R.push_back(-20000);
                    LOG(WARNING) << "Detected deep collision and aborted execution. Parameters are: " << parameters::paramVecToStream(policy.params(), true).rdbuf();
                }

                std::vector<Eigen::VectorXd> states = controller->get_states();
                std::vector<Eigen::VectorXd> noiseless_states = controller->get_noiseless_states();
                std::vector<Eigen::VectorXd> commands = controller->get_commands();
                if (display) {
                    this->_last_states = states;
                    this->_last_dense_states = controller->get_dense_states();
                    this->_last_commands = commands;
                    this->_last_dense_commands = controller->get_dense_commands();
                    this->_last_dense_ee_pos = controller->get_dense_ee_pos();
                    this->_last_dense_ee_rot = controller->get_dense_ee_rot();
                    this->_last_applied_residual_mean = controller->get_applied_residual_mean();
                    this->_last_applied_residual_variance = controller->get_applied_residual_variance();
                }
                rollout_info.finish_time = controller->finish_time;

                // Order results and calculate reward
                std::vector<Eigen::VectorXd> transformed_state;
                transformed_state.resize(states.size());
                for (size_t j = 0; j < states.size() - 1; j++) {
                    transformed_state[j] = this->transform_state(states[j]);
                    res.push_back(std::make_tuple(transformed_state[j], commands[j], states[j + 1] - states[j]));
                }
                    
                R = reward::calculate_rewards<Reward, RolloutInfo, Params>(world, rollout_info, noiseless_states, commands);

                if (!policy.random() && display) {
                    LOG(INFO) << std::boolalpha << "Solution is feasible: " << reward::check_feasibility(R);
                    LOG(INFO) << reward::rewards_to_stream(R).rdbuf();
                    rollout_info.print();
                }
                return res;
            }

            bool check_for_deep_collisions(const dart::collision::CollisionResult& col_res, double depth) {
                const auto contacts = col_res.getContacts();
                for (const auto& contact : contacts) {
                    if (contact.penetrationDepth > depth) {
                        return true;
                    }
                }
                return false;
            }

            // override this to add extra stuff to the robot_dart simulator
            virtual void add_extra_to_simu(robot_simu_t& simu, const RolloutInfo& rollout_info) const {}

            // you should override this, to define how your simulated robot_dart::Robot will be constructed
            virtual std::shared_ptr<robot_dart::Robot> get_robot() const = 0;

            // override this if you want to set in a specific way the initial state of your robot
            virtual void set_robot_state(const std::shared_ptr<robot_dart::Robot>& robot, const Eigen::VectorXd& state) const {}

            // get states from last execution
            std::vector<Eigen::VectorXd> get_last_dense_states() const
            {
                return _last_dense_states;
            }

            // get commands from last execution
            std::vector<Eigen::VectorXd> get_last_dense_commands() const
            {
                return _last_dense_commands;
            }

            // get ee pos from last execution
            std::vector<Eigen::VectorXd> get_last_dense_ee_pos() const
            {
                return _last_dense_ee_pos;
            }

            // get ee rot from last execution
            std::vector<Eigen::VectorXd> get_last_dense_ee_rot() const
            {
                return _last_dense_ee_rot;
            }

            // get applied residual mean
            std::vector<Eigen::VectorXd> get_last_applied_residual_mean() const
            {
                return _last_applied_residual_mean;
            }

            // get applied residual variance
            std::vector<Eigen::VectorXd> get_last_applied_residual_variance() const
            {
                return _last_applied_residual_variance;
            }

            void set_scene_setup(std::function<void(robot_dart::RobotDARTSimu& simu, const RolloutInfo& info)> func)
            {
                _scene_setup = func;
            }

            std::vector<Eigen::VectorXd> _last_dense_states, _last_dense_commands, _last_dense_ee_pos, _last_dense_ee_rot, _last_applied_residual_mean, _last_applied_residual_variance;
            std::shared_ptr<learning_skills_execution<PolicyController, Params>> _execution;
            std::function<void(robot_dart::RobotDARTSimu& simu, const RolloutInfo& info)> _scene_setup;
        };

        template <typename Params, typename Policy>
        class BaseDARTPolicyControl : public robot_dart::control::RobotControl {
        public:
            using robot_t = std::shared_ptr<robot_dart::Robot>;

            BaseDARTPolicyControl() {
                // set some default functions in case the user does not define them
                set_transform_state(std::bind(&BaseDARTPolicyControl::transform_state, this, std::placeholders::_1));
                set_noise_function(std::bind(&BaseDARTPolicyControl::transform_state, this, std::placeholders::_1));
                set_policy_function(std::bind(&BaseDARTPolicyControl::transform_state, this, std::placeholders::_1));
                set_update_function(std::bind(&BaseDARTPolicyControl::dummy, this, std::placeholders::_1));
            }
            BaseDARTPolicyControl(const Eigen::VectorXd& ctrl, bool full_control = false)
                : robot_dart::control::RobotControl(ctrl, full_control)
            {
                BaseDARTPolicyControl();
            }

            void configure() override
            {
                _prev_time = 0.0;
                _t = 0.0;
                _first = true;

                // _policy.set_params(_ctrl);

                _states.clear();
                _noiseless_states.clear();
                _coms.clear();

                if (Params::skireil::command_dim() == size_t(_control_dof))
                    _active = true;
            }

            void configure(const ParamVec& pv) {
                _policy.set_params(pv);
                configure();
            }

            Eigen::VectorXd calculate(double t) override
            {
                _t = t;
                _update_func(t);

                double dt = Params::skireil::dt();

                if (_first || (_t - _prev_time - dt) > -Params::dart_system::sim_step() / 2.0) {
                    Eigen::VectorXd q = this->get_state(_robot.lock());
                    _noiseless_states.push_back(q);
                    q = _add_noise(q);
                    Eigen::VectorXd commands = _policy.next(_policy_state(_tranform_state(q)), t);
                    _states.push_back(q);
                    _coms.push_back(commands);

                    ROBOT_DART_ASSERT(_control_dof == commands.size(), "BaseDARTPolicyControl: Policy output size is not the same as the control DOFs of the robot", Eigen::VectorXd::Zero(_control_dof));
                    _prev_commands = commands;
                    _prev_time = _t;
                    _first = false;
                }

                return _prev_commands;
            }

            std::vector<Eigen::VectorXd> get_states() const
            {
                return _states;
            }

            std::vector<Eigen::VectorXd> get_noiseless_states() const
            {
                return _noiseless_states;
            }

            std::vector<Eigen::VectorXd> get_commands() const
            {
                return _coms;
            }

            Eigen::VectorXd get_last_command() const
            {
                if (!this->active()) {
                    LOG(WARNING) << "Queried inactive controller for commands.";
                }
                if (!_coms.empty()) {
                    return _coms.back();
                } else {
                    return Eigen::VectorXd();
                }
            }

            void set_transform_state(std::function<Eigen::VectorXd(const Eigen::VectorXd&)> func)
            {
                _tranform_state = func;
            }

            void set_noise_function(std::function<Eigen::VectorXd(const Eigen::VectorXd&)> func)
            {
                _add_noise = func;
            }

            void set_policy_function(std::function<Eigen::VectorXd(const Eigen::VectorXd&)> func)
            {
                _policy_state = func;
            }

            void set_update_function(std::function<void(double)> func)
            {
                _update_func = func;
            }

            virtual Eigen::VectorXd get_state(const robot_t& robot) const = 0;

        protected:
            double _prev_time;
            double _t;
            bool _first;
            Eigen::VectorXd _prev_commands;
            Policy _policy;
            std::vector<Eigen::VectorXd> _coms;
            std::vector<Eigen::VectorXd> _states, _noiseless_states;
            std::function<Eigen::VectorXd(const Eigen::VectorXd&)> _tranform_state;
            std::function<Eigen::VectorXd(const Eigen::VectorXd&)> _add_noise;
            std::function<Eigen::VectorXd(const Eigen::VectorXd&)> _policy_state;
            std::function<void(double)> _update_func;

            Eigen::VectorXd transform_state(const Eigen::VectorXd& original_state) const
            {
                return original_state;
            }

            void dummy(double) const {}
        };
    } // namespace system
} // namespace skireil

#endif
