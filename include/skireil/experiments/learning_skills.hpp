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
#ifndef SKIREIL_LEARNING_SKILLS_HPP
#define SKIREIL_LEARNING_SKILLS_HPP

#include <sstream>
#include <cmath>
#include <assert.h>
#include <exception>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/constant.hpp>
#include <limbo/model/gp.hpp>
#include <limbo/model/multi_gp.hpp>
#include <limbo/model/multi_gp/parallel_lf_opt.hpp>

#include <skireil/skireil.hpp>
#include <skireil/model/gp/kernel_lf_opt.hpp>
#include <skireil/model/gp_model.hpp>
#include <skireil/system/dart_system.hpp>
#include <skireil/policy/skiros_policy.hpp>
#include <skireil/reward/reward.hpp>
#include <skireil/reward/reward_functions.hpp>
#include <skireil/utils/cmd_args.hpp>
#include <skireil/utils/utils.hpp>
#include <skireil/experiments/learning_skills_policy_control.hpp>
#include <skireil/experiments/learning_skills_scenes.hpp>
#include <skireil/experiments/learning_skills_robots.hpp>
#include <skireil/optimizer/hypermapper.hpp>
#include <skireil/parameters.hpp>

#include <ros/ros.h>

#include <glog/logging.h>

#include <boost/filesystem.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>


namespace learning_skills {

#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define radiansToDegrees(angleRadians) ((angleRadians) * 180.0 / M_PI)

using ParamVec = std::vector<std::shared_ptr<ParamBase>>;

struct Params {
    struct meta_conf {
        BO_DYN_PARAM(std::string, application_name);
        BO_DYN_PARAM(std::string, folder);
        BO_DYN_PARAM(std::string, model_folder);
        BO_DYN_PARAM(std::string, params_folder);
        BO_DYN_PARAM(std::string, traj_folder);
        BO_DYN_PARAM(std::string, opt_folder);
        BO_DYN_PARAM(std::string, existing_data_folder);
        BO_DYN_PARAM(std::string, config_file);
        BO_DYN_PARAM(int, threads);
        BO_DYN_PARAM(bool, verbose);
    };

    struct skireil : public ::skireil::defaults::skireil {
        BO_DYN_PARAM(size_t, num_params);
        BO_DYN_PARAM(ParamVec, param_vec);
        BO_DYN_PARAM(size_t, action_dim);
        BO_DYN_PARAM(size_t, command_dim);
        BO_DYN_PARAM(size_t, model_input_dim);
        BO_DYN_PARAM(size_t, model_pred_dim);
        BO_DYN_PARAM(std::vector<std::string>, state_objects);
        // Single time step for querying the action from the policy
        BO_PARAM(double, dt, 0.1);
        // Single time step to calculate control values
        BO_PARAM(double, control_dt, 0.002);
        // episode length in seconds
        BO_DYN_PARAM(double, T);
        BO_DYN_PARAM(size_t, max_fun_evals);
        BO_DYN_PARAM(double, boundary);
        BO_DYN_PARAM(bool, learn_real_system);
        // assume stochastic or deterministig episodes
        // if stochastic: there's a number of rollouts parameters
        BO_DYN_PARAM(bool, stochastic);
        BO_DYN_PARAM(bool, real_rollout);
        BO_DYN_PARAM(Eigen::VectorXd, gp_scale);
        BO_PARAM(bool, stochastic_evaluation, true);// Enables the two behaviors below.
        BO_DYN_PARAM(int, num_evals);                // How often will a fake real execution be executed?
        BO_DYN_PARAM(int, opt_evals);                // How often will optimization evaluation be executed?
        BO_DYN_PARAM(bool, domain_randomization);   // Enable domain randomization
        BO_PARAM(double, max_collision_depth, 0.08);
        BO_PARAM(double, max_joint_vel, 0.4);
        BO_DYN_PARAM(bool, nn_policy);
        BO_DYN_PARAM(std::string, optimizer);
        BO_DYN_PARAM(json, optimizer_config);
        BO_DYN_PARAM(std::vector<std::string>, objectives);
        BO_DYN_PARAM(json, reward_config);
        BO_DYN_PARAM(json, robot_init_states_config);
        BO_DYN_PARAM(json, robot_config);
        BO_DYN_PARAM(std::string, robot_end_effector);
        BO_DYN_PARAM(std::vector<std::string>, robot_dof);
        BO_DYN_PARAM(std::vector<std::string>, tool_dof);
        BO_DYN_PARAM(std::string, scene);
    };

    struct dart_system {
        BO_PARAM(double, sim_step, 0.001);
    };

    struct dart_policy_control {
        BO_PARAM_STRING(joint_type, "torque");
    };

    struct gp_model {
        BO_PARAM(double, noise, 0.0000001);
    };

    struct mean_constant {
        BO_PARAM(double, constant, 0.0);
    };

    struct kernel : public limbo::defaults::kernel {
        BO_PARAM(double, noise, gp_model::noise());
        BO_PARAM(bool, optimize_noise, true);
    };

    struct kernel_squared_exp_ard : public limbo::defaults::kernel_squared_exp_ard {
    };

    struct opt_hm {
        BO_DYN_PARAM(float, ubound);
        BO_DYN_PARAM(float, lbound);
        BO_DYN_PARAM(int, start_it);
    };

    struct opt_rprop : public limbo::defaults::opt_rprop {
        BO_PARAM(int, iterations, 300);
        BO_PARAM(double, eps_stop, 1e-4);
    };
};

struct PolicyParams {
    struct skireil : public Params::skireil {
    };

    struct bt_policy {
        BO_DYN_PARAM(bool, verbose);
        BO_DYN_PARAM(std::string, next_action_service);
        BO_DYN_PARAM(std::string, worker_manager);
        BO_DYN_PARAM(std::string, application_name);
    };

    struct nn_policy {
        BO_PARAM(size_t, state_dim, 7);
        BO_PARAM(size_t, hidden_neurons, 10);
        BO_PARAM(size_t, action_dim, 3);
        BO_PARAM_ARRAY(double, max_u, 2.0, 2.0, 2.0);
        BO_PARAM_ARRAY(double, limits, 3.0, 2.1, 3.0, 2.1, 3.0, 2.1, 3.0);
        BO_PARAM(double, af, 1.0);
    };
};

namespace global {
    std::shared_ptr<robot_dart::Robot> global_robot;

    using policy_t = skireil::policy::SkiROSPolicy<PolicyParams>;

    Eigen::VectorXd goal(3);
} // namespace global

Eigen::VectorXd get_random_vector(size_t dim, Eigen::VectorXd bounds)
{
    Eigen::VectorXd rv = (limbo::tools::random_vector(dim).array() * 2 - 1);
    // rv(0) *= 3; rv(1) *= 5; rv(2) *= 6; rv(3) *= M_PI; rv(4) *= 10;
    return rv.cwiseProduct(bounds);
}

std::vector<Eigen::VectorXd> random_vectors(size_t dim, size_t q, Eigen::VectorXd bounds)
{
    std::vector<Eigen::VectorXd> result(q);
    for (size_t i = 0; i < q; i++) {
        result[i] = get_random_vector(dim, bounds);
    }
    return result;
}

// Collects information about a rollout and can print the rewards.
struct RolloutInfo {
    Eigen::VectorXd init_state;
    Eigen::VectorXd target;
    Eigen::VectorXd goal = Eigen::VectorXd::Zero(3);
    double t{0};
    double T{0};
    double dt{0};
    double r_joints{0};
    double r_goal{0};
    double r_avoidance{0};
    double r_finish{0};
    double r_special{0};
    double finish_time{-2};

    double total_reward() {
        return r_joints + r_goal + r_avoidance + r_finish + r_special;
    }

    void print() {
        LOG(INFO) << "BT finish time:\t\t" << finish_time;
        LOG(INFO) << "Total reward combined:\t\t" << total_reward();
        LOG(INFO) << "Total reward for joints:\t\t" << r_joints;
        LOG(INFO) << "Total reward for goal:\t\t" << r_goal;
        LOG(INFO) << "Total reward for avoidance:\t" << r_avoidance;
        LOG(INFO) << "Total special reward:\t\t" << r_special;
        LOG(INFO) << "Total reward for BT success:\t" << r_finish;
    }
};

// The experiment definition
struct SimpleArm : public skireil::system::DARTSystem<Params, PolicyControl<Params, global::policy_t>, RolloutInfo> {
    using base_t = skireil::system::DARTSystem<Params, PolicyControl<Params, global::policy_t>, RolloutInfo>;

    SimpleArm() {
        _rollout_info.goal = global::goal;
        _start_pos = process_init_states(Params::skireil::robot_init_states_config());
        CHECK_GE(_start_pos.size(), 1) << "At least one start configuration needs to be defined.";
    }

    std::vector<Eigen::VectorXd> process_init_states(json j) {
        std::vector<Eigen::VectorXd> start_pos;
        for (const auto& j_i : j.items()) {
            std::vector<double> val = j_i.value();
            Eigen::VectorXd v = Eigen::VectorXd::Zero(val.size());
            for (size_t i = 0; i < val.size(); i++) {
                v[i] = val.at(i);
            }
            CHECK_LE(v.size(), Params::skireil::model_input_dim()) << "Init state needs to be smaller than model input dimension.";
            start_pos.push_back(v);
        }
        return start_pos;
    }

    // Initialize the state, e.g. joints. Additionally there's a simulation setup function.
    Eigen::VectorXd init_state() const
    {
        // This corresponds to the "set_robot_state" function below
        Eigen::VectorXd init_state(Params::skireil::model_input_dim());
        int index = 0;
        if (Params::skireil::domain_randomization()) {
            Eigen::VectorXd r_nr_v = skireil::utils::uniform_rand(1);
            double r_nr = r_nr_v[0] * _start_pos.size();
            // The range for index 0 shall be from -0.5 to 0.5.
            r_nr -= 0.5;
            index = static_cast<int>(std::round(r_nr));
            // Cover edge cases:
            if (index < 0) {
                index = 0;
            } else if (index >= (int)_start_pos.size()) {
                index = static_cast<int>(_start_pos.size()) - 1;
            }
            if (Params::meta_conf::verbose()) {
                LOG(INFO) << "Start position number [1-" << _start_pos.size() << "]: " << index + 1;
            }
        }
        // The last entries will be zeros. E.g. for joint velocities.
        init_state << _start_pos.at(index), Eigen::VectorXd::Zero(Params::skireil::model_input_dim() - _start_pos.at(index).size());
        return init_state;
    }

    // Receive state and transform it to feed it to the GP
    // Eigen::VectorXd transform_state(const Eigen::VectorXd& original_state) const
    // {
    //     Eigen::VectorXd ret = Eigen::VectorXd::Zero(Params::skireil::model_input_dim());
    //     for (int j = 0; j < original_state.size(); j++) {
    //         ret(2 * j) = std::cos(original_state(j));
    //         ret(2 * j + 1) = std::sin(original_state(j));
    //     }

    //     return ret;
    // }

    //Eigen::VectorXd add_noise(const Eigen::VectorXd& original_state) const
    //{
        // Code to add observation noise to the system
        // you should return the full noisy state, not just the noise
        // if no noise is desired, just return the original state
        // if you omit this function, no noise is added
    //}

    // Eigen::VectorXd policy_transform(const Eigen::VectorXd& original_state, RolloutInfo* info) const
    // {
    //     // Code to transform the state variables that go to the policy if needed
    //     // the input original_state is the transformed state (by the transform_state variable)
    // }


    // Returns a fixed robot
    std::shared_ptr<robot_dart::Robot> get_robot() const
    {
        std::shared_ptr<robot_dart::Robot> simulated_robot = global::global_robot->clone();
        simulated_robot->fix_to_world();
        simulated_robot->set_position_enforced(true);
        return simulated_robot;
    }

    // if you want, you can add some extra to your simulator object (this is called once before its episode on a newly-created simulator object)
    void add_extra_to_simu(base_t::robot_simu_t& simu, const RolloutInfo& info) const
    {
        _scene_setup(simu, info);
    }

    void set_robot_state(const std::shared_ptr<robot_dart::Robot>& robot, const Eigen::VectorXd& state) const
    {
        robot->set_positions(state.head(7), Params::skireil::robot_dof());
        robot->set_velocities(state.segment(7,7), Params::skireil::robot_dof());
    }

    private:
        std::vector<Eigen::VectorXd> _start_pos;
};

using scene_function_t = std::function<void(robot_dart::RobotDARTSimu& simu, const RolloutInfo& info)>;
using scenes_map_t = std::map<std::string, scene_function_t>;

template <typename RolloutInfo, typename Params>
scenes_map_t get_scenes_map() {
    scenes_map_t scenes_map;
    scenes_map["obstacle_and_peg"] = &scenes::scene_obstacle_and_peg<RolloutInfo, Params>;
    scenes_map["peg"] = &scenes::scene_peg<RolloutInfo, Params>;
    scenes_map["polyhedron"] = &scenes::scene_polyhedron<RolloutInfo, Params>;
    return scenes_map;
}

template <typename RolloutInfo, typename Params>
scene_function_t get_scene(std::string name) {
    scenes_map_t sm = get_scenes_map<RolloutInfo, Params>();
    if (sm.find(name) == sm.end()) {
        LOG(FATAL) << "Could not find scene '" << name << "' in map.";
    } else {
        return sm[name];
    }
}

using robot_setup_t = std::function<std::shared_ptr<robot_dart::Robot>(const json& j)>;
using robot_map_t = std::map<std::string, robot_setup_t>;

robot_map_t get_robot_map() {
    robot_map_t robot_map;
    robot_map["bh_rss"] = &robots::bh_rss<Params>;
    robot_map["bh_rss_2f_gripper"] = &robots::bh_rss_2f_gripper<Params>;
    robot_map["bh_rss_polyhedron"] = &robots::bh_rss_polyhedron<Params>;
    return robot_map;
}

std::shared_ptr<robot_dart::Robot> get_robot (const json& j) {
    LOG(INFO) << j;
    robot_map_t rm = get_robot_map();
    if (!j.contains("setup_name")) {
        LOG(FATAL) << "Field 'setup_name' in robot description needs to be supplied.";
    }
    std::string sn {j["setup_name"].get<std::string>()};
    if (rm.find(sn) == rm.end()) {
        LOG(FATAL) << "Could not find robot setup '" << sn << "' in map.";
    }
    return rm[sn](j);
}

// Called even before the experiment is set up.
void init_simu(const json& rs)
{
    global::global_robot = get_robot(rs);

    // The following lines can be used to set a goal position using a joint configuration of the robot arm
    // std::shared_ptr<robot_dart::Robot> simulated_robot = global::global_robot->clone();
    // simulated_robot->fix_to_world();
    // simulated_robot->set_position_enforced(true);
    // // here set goal position joint configuration
    // Eigen::VectorXd positions(7);
    // positions << 0, 0, 0, -M_PI / 2., 0, 0, 0;
    // simulated_robot->skeleton()->setPositions(positions);

    //auto bd = simulated_robot->skeleton()->getBodyNode("bh_link_ee");
    //global::goal = bd->getTransform().translation();
    Eigen::VectorXd goal_pos (3);
    goal_pos << -0.6, 0.0, 0.7;
    global::goal = goal_pos;

    LOG(INFO) << "Goal is at: " << global::goal.transpose();
}


template <typename T>
void param_from_json(const json& j, std::function<void (T)> set, std::string name) {
    if (!j.contains(name)) {
        std::stringstream s;
        s << "Param '" << name << "' does not exist in JSON.";
        throw std::invalid_argument(s.str());
    }
    try {
        set(j[name].get<T>());
    } catch (const std::exception& e) {
        std::cerr << "Error when processing param '" << name << ". " << e.what() << std::endl;
    }
    return;
}

bool process_configuration(skireil::utils::CmdArgs& cmd_arguments) {
    //
    // Parse things and set parameters
    //
    Params::skireil::set_learn_real_system(cmd_arguments.learn_real());
    if (Params::skireil::learn_real_system()) {
        LOG(INFO) << "Learning on the real system. Setting number of threads to 1.";
        Params::meta_conf::set_threads(1);
    } else {
        Params::meta_conf::set_threads(cmd_arguments.threads());
    }

    PolicyParams::bt_policy::set_next_action_service("/skiros_worker_");
    PolicyParams::bt_policy::set_worker_manager("/skiros_worker_manager");
    PolicyParams::bt_policy::set_verbose(cmd_arguments.verbose());
    // PolicyParams::gp_policy::set_pseudo_samples(cmd_arguments.pseudo_samples());

    Params::skireil::set_nn_policy(cmd_arguments.neural_net());
    Params::skireil::set_boundary(cmd_arguments.boundary());
    Params::skireil::set_max_fun_evals(cmd_arguments.max_fun_evals());
    Params::skireil::set_optimizer("hypermapper");
    Params::opt_hm::set_lbound(-cmd_arguments.boundary());
    Params::opt_hm::set_ubound(cmd_arguments.boundary());

    Params::meta_conf::set_verbose(cmd_arguments.verbose());
    Params::skireil::set_stochastic(cmd_arguments.stochastic());
    Params::skireil::set_real_rollout(cmd_arguments.real_rollout());
    Params::skireil::set_domain_randomization(true);

    json j = fileToJson(cmd_arguments.config());
    Params::meta_conf::set_config_file(cmd_arguments.config());
    if (Params::meta_conf::verbose()) {
        std::cout << j.dump(4);
    }
    param_from_json<std::string>(j, Params::meta_conf::set_application_name, "application_name");
    PolicyParams::bt_policy::set_application_name(Params::meta_conf::application_name());
    param_from_json<int>(j, Params::meta_conf::set_threads, "threads");
    param_from_json<size_t>(j, Params::skireil::set_max_fun_evals, "max_fun_evals");
    param_from_json<double>(j, Params::skireil::set_T, "episode_length");
    if (!j.contains("learning_platform")) {
        std::stringstream s;
        s << "Param 'learning_platform' does not exist in JSON.";
        throw std::invalid_argument(s.str());
    }
    if (j["learning_platform"].get<std::string>() == "real"){
        Params::skireil::set_learn_real_system(true);
    } else {
        Params::skireil::set_learn_real_system(false);
    }
    param_from_json<bool>(j, Params::skireil::set_domain_randomization, "domain_randomization");
    param_from_json<std::string>(j, Params::skireil::set_optimizer, "optimizer");
    param_from_json<json>(j, Params::skireil::set_optimizer_config, "optimizer_config");
    param_from_json<json>(j, Params::skireil::set_reward_config, "rewards");
    param_from_json<json>(j, Params::skireil::set_robot_init_states_config, "robot_init_states");
    param_from_json<json>(j, Params::skireil::set_robot_config, "robot");
    param_from_json<int>(j, Params::skireil::set_num_evals, "expected_reward_evals");
    param_from_json<int>(j, Params::skireil::set_opt_evals, "evals_per_param_config");
    param_from_json<std::string>(j, Params::skireil::set_scene, "scene");

    #ifdef USE_TBB
    static tbb::task_scheduler_init init(Params::meta_conf::threads());
    #endif
    return true;
}

bool process_configuration(int argc, char** argv) {
    // Start with command line arguments
    skireil::utils::CmdArgs cmd_arguments;
    int ret = cmd_arguments.parse(argc, argv);
    if (ret >= 0)
        return ret;
    return process_configuration(cmd_arguments);
}


template <typename reward_t>
std::shared_ptr<reward_t> process_reward_config(const json& j, const std::map<std::string, std::shared_ptr<skireil::reward::Reward<RolloutInfo>>(*)()>& reward_function_map, std::shared_ptr<robot_dart::Robot> robot) {
    std::shared_ptr<reward_t> reward_functions = std::make_shared<reward_t>(reward_t());
    for (const auto& j_i : j.items()) {
        std::cout << j_i.key() << ": " << j_i.value() << std::endl;
        if (!j_i.value().contains("type")) {
            LOG(FATAL) << "Reward '" << j_i.key() << "': Type is not specified. Aborting";
        }
        std::string type = j_i.value()["type"].get<std::string>();
        auto it = reward_function_map.find(type);
        if (it == reward_function_map.end()) {
            LOG(ERROR) << "Reward '" << j_i.key() << "': Function '" << type << "' could not be found in the map of reward functions. Skipping.";
            continue;
        }
        auto rew = (*it).second();
        rew->set_name(type);
        rew->set_key(j_i.key());
        std::string obj;
        if (!j_i.value().contains("objective")) {
            LOG(WARNING) << "Reward '" << j_i.key() << "': Objective is not specified. Assuming 'reward'.";
            obj = "reward";
        } else {
            obj = j_i.value()["objective"].get<std::string>();
        }
        if (!j_i.value().contains("weight")) {
            LOG(INFO) << "Reward '" << j_i.key() << "': Weight of reward function not specified. Assuming 1.0.";
        } else {
            rew->set_weight(j_i.value()["weight"].get<double>());
        }
        if (!rew->configure(j_i.value(), CHECK_NOTNULL(robot), Params::skireil::robot_dof())) {
            LOG(ERROR) << "Reward function '" << j_i.key() << "': Configuration failed. Skipping.";
            continue;
        }
        reward_functions->insert(std::pair<std::string, std::shared_ptr<skireil::reward::Reward<RolloutInfo>>>(obj, rew));

    }
    LOG_IF(WARNING, reward_functions->size() == 0) << "Reward map is empty.";
    return reward_functions;
}

bool init_folder(std::string folder, std::string suffix = std::string()) {
    if (boost::filesystem::create_directories(folder)) {
        // If the folder was already created it might have been another user and this would fail
        boost::filesystem::permissions(folder, boost::filesystem::add_perms|boost::filesystem::all_all);
    }

    // Multi-iteration processes can restart from a given data folder. Otherwise create a new one.
    if (!Params::meta_conf::existing_data_folder().empty()) {   
        if (!boost::filesystem::is_directory(Params::meta_conf::existing_data_folder())) {
            std::cerr << "Fatal: Specified data location " << Params::meta_conf::existing_data_folder() << " is not a directory." << std::endl;
            return false;
        }
        folder = Params::meta_conf::existing_data_folder();     
    } else {
        const std::time_t t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::stringstream timestr;
        timestr << std::put_time(std::localtime(&t), "%Y%m%d-%H%M%S");
        folder += timestr.str() + "_" + Params::meta_conf::application_name() + suffix;
    }
    if (folder.back() != std::string("/").back()) {
        folder += "/";
    }
    Params::meta_conf::set_folder(folder);
    Params::meta_conf::set_model_folder(folder+"models/");
    Params::meta_conf::set_params_folder(folder+"parameters/");
    Params::meta_conf::set_traj_folder(folder+"trajectories/");
    Params::meta_conf::set_opt_folder(folder+"optimizations/");
    boost::filesystem::create_directories(Params::meta_conf::model_folder());
    boost::filesystem::create_directories(Params::meta_conf::params_folder());
    boost::filesystem::create_directories(Params::meta_conf::traj_folder());
    boost::filesystem::create_directories(Params::meta_conf::opt_folder());
    return true;
}

void write_parameters_to_log() {
    LOG(INFO) << "Blackdrops parameters:";
    LOG(INFO) << "  Stochastic rollouts = " << Params::skireil::stochastic();
    LOG(INFO) << "  max_fun_evals = " << Params::skireil::max_fun_evals();
    LOG(INFO) << "  Boundary = " << Params::skireil::boundary();
    LOG(INFO) << "  TBB threads = " << Params::meta_conf::threads();
    LOG(INFO) << "  Iterations = " << Params::skireil::max_fun_evals();
    LOG(INFO) << "  Do a real rollout = " << Params::skireil::real_rollout();
    LOG(INFO) << "  Learn on real hardware = " << Params::skireil::learn_real_system();
    LOG(INFO) << "  Domain Randomization = " << Params::skireil::domain_randomization();
    LOG(INFO) << "  Maximum collision depth = " << Params::skireil::max_collision_depth();

    int N = (Params::skireil::stochastic_evaluation()) ? Params::skireil::opt_evals() : 1;
    LOG(INFO) << "  Rolling out policy " << N << " times.";
}

void init_logging(char** argv) {
    google::SetLogDestination(google::GLOG_INFO, std::string(Params::meta_conf::folder()+"INFO_").c_str());
    google::SetLogDestination(google::GLOG_WARNING, std::string(Params::meta_conf::folder()+"WARNING_").c_str());
    google::SetLogDestination(google::GLOG_ERROR, std::string(Params::meta_conf::folder()+"ERROR_").c_str());
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = 1;
}

BO_DECLARE_DYN_PARAM(std::string, Params::meta_conf, application_name);
BO_DECLARE_DYN_PARAM(bool, Params::meta_conf, verbose);
BO_DECLARE_DYN_PARAM(std::string, Params::meta_conf, folder);
BO_DECLARE_DYN_PARAM(std::string, Params::meta_conf, model_folder);
BO_DECLARE_DYN_PARAM(std::string, Params::meta_conf, params_folder);
BO_DECLARE_DYN_PARAM(std::string, Params::meta_conf, traj_folder);
BO_DECLARE_DYN_PARAM(std::string, Params::meta_conf, opt_folder);
BO_DECLARE_DYN_PARAM(std::string, Params::meta_conf, existing_data_folder);
BO_DECLARE_DYN_PARAM(std::string, Params::meta_conf, config_file);
BO_DECLARE_DYN_PARAM(int, Params::meta_conf, threads);

BO_DECLARE_DYN_PARAM(std::string, PolicyParams::bt_policy, next_action_service);
BO_DECLARE_DYN_PARAM(std::string, PolicyParams::bt_policy, worker_manager);
BO_DECLARE_DYN_PARAM(std::string, PolicyParams::bt_policy, application_name);
BO_DECLARE_DYN_PARAM(bool, PolicyParams::bt_policy, verbose);

BO_DECLARE_DYN_PARAM(size_t, Params::skireil, num_params);
BO_DECLARE_DYN_PARAM(ParamVec, Params::skireil, param_vec);
BO_DECLARE_DYN_PARAM(size_t, Params::skireil, action_dim);
BO_DECLARE_DYN_PARAM(size_t, Params::skireil, command_dim);
BO_DECLARE_DYN_PARAM(size_t, Params::skireil, model_input_dim);
BO_DECLARE_DYN_PARAM(size_t, Params::skireil, model_pred_dim);
BO_DECLARE_DYN_PARAM(std::vector<std::string>, Params::skireil, state_objects);
BO_DECLARE_DYN_PARAM(double, Params::skireil, boundary);
BO_DECLARE_DYN_PARAM(double, Params::skireil, T);
BO_DECLARE_DYN_PARAM(size_t, Params::skireil, max_fun_evals);
BO_DECLARE_DYN_PARAM(int, Params::skireil, num_evals);
BO_DECLARE_DYN_PARAM(int, Params::skireil, opt_evals);
BO_DECLARE_DYN_PARAM(bool, Params::skireil, learn_real_system);
BO_DECLARE_DYN_PARAM(bool, Params::skireil, stochastic);
BO_DECLARE_DYN_PARAM(bool, Params::skireil, real_rollout);
BO_DECLARE_DYN_PARAM(Eigen::VectorXd, Params::skireil, gp_scale);
BO_DECLARE_DYN_PARAM(bool, Params::skireil, domain_randomization);
BO_DECLARE_DYN_PARAM(bool, Params::skireil, nn_policy);
BO_DECLARE_DYN_PARAM(std::string, Params::skireil, optimizer);
BO_DECLARE_DYN_PARAM(json, Params::skireil, optimizer_config);
BO_DECLARE_DYN_PARAM(std::vector<std::string>, Params::skireil, objectives);
BO_DECLARE_DYN_PARAM(json, Params::skireil, reward_config);
BO_DECLARE_DYN_PARAM(json, Params::skireil, robot_init_states_config);
BO_DECLARE_DYN_PARAM(json, Params::skireil, robot_config);
BO_DECLARE_DYN_PARAM(std::string, Params::skireil, robot_end_effector);
BO_DECLARE_DYN_PARAM(std::vector<std::string>, Params::skireil, robot_dof);
BO_DECLARE_DYN_PARAM(std::vector<std::string>, Params::skireil, tool_dof);
BO_DECLARE_DYN_PARAM(std::string, Params::skireil, scene);

BO_DECLARE_DYN_PARAM(float, Params::opt_hm, lbound);
BO_DECLARE_DYN_PARAM(float, Params::opt_hm, ubound);
BO_DECLARE_DYN_PARAM(int, Params::opt_hm, start_it);

} // namespace learning_skills

#endif
