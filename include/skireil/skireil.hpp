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

#ifndef SKIREIL_SKIREIL_HPP
#define SKIREIL_SKIREIL_HPP

#include <chrono>
#include <thread>
#include <Eigen/binary_matrix.hpp>
#include <chrono>
#include <fstream>
#include <limbo/opt/optimizer.hpp>
#include <limits>
#include <skireil/utils/utils.hpp>
#include <skireil/parameters.hpp>
#include <skireil/reward/reward_result.hpp>

#include <glog/logging.h>

Eigen::IOFormat OneLine(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", ";");
Eigen::IOFormat OneLineWoTerm(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");

using ParamVec = std::vector<std::shared_ptr<skireil::parameters::ParamBase>>;
using rollout_result = std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>>;
// transformed_state, u, diff_to_next_state

namespace skireil {

    namespace defaults {
        struct skireil {
            BO_PARAM(bool, stochastic_evaluation, false);
            BO_PARAM(int, num_evals, 0);
            BO_PARAM(int, opt_evals, 1);
        };
    } // namespace defaults

    void print_rollout_result(const rollout_result& result, std::string prefix_state = "", std::string prefix_next = ""){
        for (size_t i = 0; i < result.size(); i++) {
            LOG(INFO) << prefix_state << "State before: " << std::get<0>(result[i]).format(OneLine);
            LOG(INFO) << "Action: " << std::get<1>(result[i]).format(OneLine);
            LOG(INFO) << prefix_next << "State after: " << std::get<2>(result[i]).format(OneLine);
        }
    }

    struct EvalResult {
        EvalResult(double r_, unsigned int it_, const ParamVec& params_ ) :
            r{r_}, it{it_}, params{params_} {}

        std::string c_sep() const {
            return std::to_string(it) + "," + std::to_string(r) + "," + skireil::parameters::paramVecToStream(params, true).str();
        }

        std::string pretty_print(bool spaces = true, bool worse = false, bool close = false) const {
            std::stringstream out;
            if (spaces) {
                out << "       ";
            }
            if (worse) {
                out << "Worse: ";
            }
            if (close) {
                out << "Close: ";
            }
            out << "(" << it << ", " << r << ")";
            out << "      Params: " << parameters::paramVecToStream(params, true).rdbuf();
            return out.str();
        }
        const double r{0};
        const unsigned int it{0};
        const ParamVec params;
    };

    struct MeanEvaluator {
        double operator()(const Eigen::VectorXd& rews) const { return rews.mean(); }
    };

    template <typename Params, typename Model, typename Robot, typename Policy, typename PolicyOptimizer, typename Reward, typename RolloutInfo, typename Evaluator = MeanEvaluator>
    class BlackDROPS {
    public:
        BlackDROPS() : _best(-std::numeric_limits<double>::max()) {}
        BlackDROPS(const PolicyOptimizer& optimizer) : _policy_optimizer(optimizer), _best(-std::numeric_limits<double>::max()) {}
        ~BlackDROPS() {}

        // Checks if the asynchronous execution on the real system has happened based on the existence of the trajectory file.
        // Reads the file and processes the results.
        bool process_real_execution_results(std::string filename, rollout_result& obs_new, RolloutInfo& rollout_info) {
            std::ifstream result_file(filename);
            if(!result_file.fail()) {
                LOG(INFO) << "Found rollout observation. Continuing...";
                std::vector<Eigen::VectorXd> states;
                std::string line;

                while (std::getline(result_file, line)) {
                    std::stringstream ss;
                    ss.str(line);
                    std::string time_key = std::string("finish_time:");
                    if (line.find(time_key) != std::string::npos) {
                        rollout_info.finish_time = std::stod(line.substr(time_key.size()));
                        LOG(INFO) << "Rollout finished in " << rollout_info.finish_time << " s.";
                    } else {
                        Eigen::VectorXd state(Params::skireil::model_pred_dim());
                        Eigen::VectorXd action(Params::skireil::action_dim());
                        for (size_t i = 0; i < Params::skireil::model_pred_dim(); i++) {
                            double c_s;
                            ss >> c_s;
                            state[i] = c_s;
                        }
                        for (size_t i = 0; i < Params::skireil::action_dim(); i++) {
                            double c_a;
                            ss >> c_a;
                            action[i] = c_a;
                        }
                        if (ss.fail() || ss.eof()) {
                            LOG(ERROR) << "Failure when reading from file: " << filename;
                            LOG(ERROR) << "Current line: " << line;
                        }
                        obs_new.push_back(std::make_tuple(state, action, Eigen::VectorXd(Params::skireil::model_pred_dim())));
                    }
                }
                return true;
            } else {
                return false;
            }
        }

        // Execution on the real robot
        void execute_and_record_data()
        {
            LOG(INFO) << "Parameters for execution on real system: " << parameters::paramVecToStream(_policy.params(), false).rdbuf();

            // Execute best policy so far on robot
            rollout_result obs_new;
            reward::RewardResultMap rewards;
            if (Params::skireil::real_rollout()) {
                std::string param_filename(Params::meta_conf::params_folder() + "policy_params_" + std::to_string(_iteration) + ".json");
                parameters::paramVecToFile(_policy.params(), param_filename);
                parameters::paramVecToFile(_policy.params(), "/tmp/policy_params.json");
                LOG(INFO) << "Dumped parameters to: " << param_filename;
                LOG(INFO) << "\n\n\n\n\n\tIT'S ABOUT TIME TO DO A ROLLOUT MY FRIEND\n\n\n\n\n";
                LOG(INFO) << "Expecting rollout observation at: " << _ofs_real_name;
                RolloutInfo rollout_info;
                while (!process_real_execution_results(_ofs_real_name, obs_new, rollout_info)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                }
                // Calculate rewards
                std::vector<Eigen::VectorXd> states;
                std::vector<Eigen::VectorXd> commands;
                for (const auto& e : obs_new) {
                    states.push_back(std::get<0>(e));
                    states.push_back(std::get<1>(e));
                }

                rewards = reward::calculate_rewards<Reward, RolloutInfo, Params>(*_reward, rollout_info, states, commands);
                LOG(INFO) << reward::rewards_to_stream(rewards).rdbuf();
            } else {
                LOG(INFO) << "Real execution disabled. Doing a fake real execution to record data.";
                obs_new = _robot.execute(_policy, *_reward, Params::skireil::T(), rewards);
                // //Debugging: Modifies observation, so the GP needs to learn something.
                // //This can be used to shift values outside of the normal state space.
                // for (size_t i = 0; i < obs_new.size(); i++) {
                //     Eigen::VectorXd state_vec = std::get<0>(obs_new[i]);
                //     for (int j = 0; j < state_vec.size(); j++) {
                //         state_vec[j] = state_vec[j] + 10;
                //     }
                //     std::get<0>(obs_new[i]) = state_vec;
                // }
                if (Params::skireil::stochastic_evaluation()) {
                    // This is threaded!
                    std::vector<reward::RewardResultMap> rews;
                    CHECK_GE(Params::skireil::num_evals(), 1) << "Need at least 1 final evaluation.";
                    rews.resize(Params::skireil::num_evals());
                    rews.at(0) = rewards;
                    limbo::tools::par::loop(1, Params::skireil::num_evals(), [&](size_t i) {
                        // Policy objects are not thread-safe usually
                        Policy p;
                        p.set_params(_policy.params());
                        if (_policy.random())
                            p.set_random_policy();

                        reward::RewardResultMap R_more;
                        _robot.execute(p, *_reward, Params::skireil::T(), R_more, false);

                        rews.at(i) = R_more;
                    });
                    rewards = reward::reward_result_map_from_vec(rews);
                }
                LOG(INFO) << "Expected Reward: " << reward::rewards_to_stream(rewards).rdbuf() << std::endl;
                // statistics for trajectories
                utils::save_traj_to_file(_ofs_real_name, _robot.get_last_states(), _robot.get_last_commands());
                utils::save_traj_to_file(_ofs_real_ee_name, _robot.get_last_dense_ee_pos(), _robot.get_last_dense_ee_rot());
            }
            rollout_result obs_sim = _robot.template execute_with_sequence<Model>(obs_new, _policy.params(), false, _model);
            rollout_result obs_diff = calculate_diff_sim_real(obs_sim, obs_new);
            // for (size_t i = 0; i < obs_diff.size(); i++) {
            //     Eigen::VectorXd state_vec = std::get<2>(obs_diff[i]);
            //     // // Debugging: This loop can be used to introduce a difference:
            //     // // Can or can not be combined with the one above.
            //     // for (int j = 0; j < state_vec.size(); j++) {
            //     //     state_vec[j] = state_vec[j] + 1;
            //     // }
            //     // We scale the input. Remember to divice by this scale when predicting with the GPs.
            //     //state_vec.array() *= Params::skireil::gp_scale().array();
            //     std::get<2>(obs_diff[i]) = state_vec;
            // }

            // Check if it is better than the previous best if we are in a single-objective setting
            if (utils::getKeyCount(rewards) == 1) {
                double r_new {0.};
                reward::get_reward_by_objective(rewards, rewards.cbegin()->first);
                if (r_new > _best) {
                    _best = r_new;
                    _params_starting = _policy.params();
                }
                // statistics for cumulative reward (both observed and expected)
                _ofs_results << r_new << std::endl;
            }

            // Append recorded difference data
            _observations.insert(_observations.end(), obs_diff.begin(), obs_diff.end());
        }

        // Learns the model with the current observations.
        void learn_model()
        {
            if (Params::meta_conf::verbose()) {
                print_rollout_result(_observations, "", "_observation ");
            }
            _model.learn(_observations);
        }

        rollout_result calculate_diff_sim_real(const rollout_result& sim, const rollout_result& real) {
            rollout_result diff;
            for (size_t i = 0; i < sim.size() -1; i++) {
                Eigen::VectorXd state_diff(Params::skireil::model_pred_dim());
                // Learning target is x_(r,t+1) - x_(s,t+1)
                state_diff = std::get<0>(sim[i+1])-std::get<2>(sim[i]);
                diff.push_back(std::make_tuple(std::get<0>(sim[i]), std::get<1>(sim[i]), state_diff));
            }
            if (Params::meta_conf::verbose()) {
                print_rollout_result(sim, "Real ", "Sim ");
                print_rollout_result(diff, "Real ", "Diff sim-real ");
            }
            return diff;
        }

        // Optimize function called by the "learn" method.
        // Calls the policy optimizer with the "internal" optimize function.
        // Pre- and postprocesses the results
        int optimize_policy(size_t i)
        {
            ParamVec params_star;
            ParamVec params_starting = _params_starting;
            LOG(INFO) << "Parameters when starting: Size: " << parameters::paramVecToStream(params_starting, true).rdbuf();
            parameters::paramVecToFile(params_starting, Params::meta_conf::params_folder() + "policy_params_" + std::to_string(i) + "_opt_before.json");

            _opt_iters = 0;
            _model_evals = 0;
            _max_reward = -std::numeric_limits<double>::max();
            _best_evals.clear();
            _all_evals.clear();
            if (_boundary == 0) {
                LOG(INFO) << "Running policy optimization with no boundaries... " << std::flush;
                params_star = _policy_optimizer(
                    std::bind(&BlackDROPS::_optimize_policy, this, std::placeholders::_1, std::placeholders::_2),
                    params_starting,
                    false);
            }
            else {
                LOG(INFO) << "Running policy optimization bounded to [-" << _boundary << ", " << _boundary << "]... " << std::flush;
                params_star = _policy_optimizer(
                    std::bind(&BlackDROPS::_optimize_policy, this, std::placeholders::_1, std::placeholders::_2),
                    params_starting,
                    true);
            }
            LOG(INFO) << "Finished. Did " << _opt_iters << " iterations.";
            if (Params::skireil::objectives().size() == 1) {
                LOG(INFO) << "Best reward seen in optimization : " << _max_reward;
                                                LOG(INFO)
                                                << "Best parameters: "
                                                << paramVecToStream(params_star, true).rdbuf();
                _policy.set_params(params_star);

                
    #ifdef GRAPHIC
                LOG(INFO) << "Visualizing policy...";
    #endif
                // // Debugging: Here we can manually introduce working parameters:
                // Eigen::VectorXd manual_params(8);
                // manual_params << 1.2,0.35,-0.2,1.3,0.85,1.3,0.4,0.85;
                // _policy.set_params(manual_params);
                LOG(INFO) << "Expected reward:";
                reward::RewardResultMap R;
                // We never do a stochastic rollout when visualizing the policy.
                _robot.execute_with_model(_policy, _model, *_reward, Params::skireil::T(), R, true, false);
                double r_eval{0};
                if (Params::skireil::stochastic_evaluation()) {
                    // This is threaded!
                    std::vector<reward::RewardResultMap> rews;
                    CHECK_GE(Params::skireil::num_evals(), 1) << "Need at least 1 final evaluation.";
                    rews.resize(Params::skireil::num_evals());
                    rews.at(0) = R;
                    limbo::tools::par::loop(1, Params::skireil::num_evals(), [&](size_t i) {
                        // Policy objects are not thread-safe usually
                        Policy p;
                        p.set_params(_policy.params());
                        if (_policy.random())
                            p.set_random_policy();

                        reward::RewardResultMap R_more;
                        _robot.execute_with_model(p, _model, *_reward, Params::skireil::T(), R_more, true, false);

                        rews.at(i) = R_more;
                    });
                    R = reward::reward_result_map_from_vec(rews);
                }
                LOG(INFO) << "Expected Reward: " << reward::rewards_to_stream(R).rdbuf();
                _ofs_exp << r_eval << std::endl;

                LOG(INFO) << "The last real execution received a reward of " << _best;
                LOG(INFO) << "Parameters after optimization: " << parameters::paramVecToStream(_policy.params(), false).rdbuf();
                parameters::paramVecToFile(_policy.params(), Params::meta_conf::params_folder() + "policy_params_" + std::to_string(i) + "_opt_final.json");

                // statistics for trajectories
                std::string traj_state = std::string(Params::meta_conf::traj_folder() + "traj_opt_final_" + std::to_string(i) + ".dat");
                std::string residual_filename = std::string(Params::meta_conf::traj_folder() + "residual_" + std::to_string(i) + ".dat");
                utils::save_traj_to_file(traj_state, _robot.get_last_states(), _robot.get_last_commands());
                utils::save_traj_to_file(_ofs_opt_ee_name, _robot.get_last_dense_ee_pos(), _robot.get_last_dense_ee_rot());
                utils::save_traj_to_file(residual_filename, _robot.get_last_applied_residual_mean(), _robot.get_last_applied_residual_variance());
            }

            // for (auto r : R)
            //     _ofs_esti << r << " ";
            // _ofs_esti << std::endl;
            return _opt_iters;
        }

        // Main function for learning new policies.
        // init: start iterations. Usually 0 unless we restart a previous process
        // iterations: number of learning iterations
        // random_policies: execute random policies to obtain data
        void learn(size_t init, size_t iterations, bool random_policies = false)
        {
            _boundary = Params::skireil::boundary();
            _random_policies = random_policies;
            _ofs_results.open(Params::meta_conf::folder() + "results.dat", std::ofstream::out | std::ofstream::app);
            _ofs_exp.open(Params::meta_conf::folder() + "expected.dat", std::ofstream::out | std::ofstream::app);
            _ofs_real.open(Params::meta_conf::folder() + "real.dat", std::ofstream::out | std::ofstream::app);
            _ofs_esti.open(Params::meta_conf::folder() + "estimates.dat", std::ofstream::out | std::ofstream::app);
            _ofs_opt.open(Params::meta_conf::folder() + "times.dat", std::ofstream::out | std::ofstream::app);
            _ofs_model.open(Params::meta_conf::folder() + "times_model.dat", std::ofstream::out | std::ofstream::app);
            _policy.set_random_policy();
            _policy.set_robot(_robot.get_robot());
            _params_starting = Params::skireil::param_vec();

            std::chrono::steady_clock::time_point time_start;
            std::chrono::steady_clock::time_point time_start_learning = std::chrono::steady_clock::now();
            LOG(INFO) << "Starting learning...";
            for (size_t i = init; i <= iterations; i++) {
                LOG(INFO) << "=========================================================================";
                LOG(INFO) << "Learning iteration # " << i << " of " << iterations;
                _iteration = i;
                _ofs_opt_ee_name = Params::meta_conf::traj_folder() + "traj_opt_final_ee_" + std::to_string(_iteration) + ".dat";
                if (i > 0) {
                    LOG(INFO) << "Executing on real system";
                    _ofs_real_name = Params::meta_conf::traj_folder() + "traj_real_" + std::to_string(i-1) + ".dat";
                    _ofs_real_ee_name = Params::meta_conf::traj_folder() + "traj_real_ee_" + std::to_string(i-1) + ".dat";
                    execute_and_record_data();

                    LOG(INFO) << "Learning model with: " << _ofs_real_name;
                    time_start = std::chrono::steady_clock::now();
                    learn_model();
                    double learn_model_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
                    _ofs_model << (learn_model_ms * 1e-3) << std::endl;
                    _model.save_model(i, Params::meta_conf::model_folder());
                    LOG(INFO) << "Learned model in: " << learn_model_ms * 1e-3 << "s" << std::endl;

                    time_start = std::chrono::steady_clock::now();
                    // Reward learning if needed
                    // if (_reward.learn()) {
                    //     double learn_reward_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
                    //     LOG(INFO) << "Learned reward in: " << learn_reward_ms * 1e-3 << "s" << std::endl;
                    // }
                }
                LOG(INFO) << "Optimizing policy";
                time_start = std::chrono::steady_clock::now();
                int opt_iterations = optimize_policy(i);
                double optimize_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start).count();
                LOG(INFO) << "Optimization time in: " << optimize_ms * 1e-3 << "s" << std::endl;
                _ofs_opt << (optimize_ms * 1e-3) << " " << _model_evals << std::endl;
                LOG(INFO) << "Optimization iterations: " << opt_iterations;
                LOG(INFO) << "Time per episode: " << (optimize_ms/opt_iterations) << " ms";
                utils::save_vec_to_file(Params::meta_conf::opt_folder() + "all_evals_" + std::to_string(_iteration) + ".dat", _all_evals);
                utils::save_vec_to_file(Params::meta_conf::opt_folder() + "best_evals_" + std::to_string(_iteration) + ".dat", _best_evals);
            }
            if (Params::skireil::objectives().size() == 1) {
            LOG(INFO) << "Executing the final policy on real system";
                _ofs_real_name = Params::meta_conf::traj_folder() + "traj_real_" + std::to_string(_iteration) + ".dat";
                execute_and_record_data();
            }
            _ofs_real.close();
            _ofs_esti.close();
            _ofs_opt.close();
            _ofs_model.close();
            _ofs_results.close();
            _ofs_exp.close();
            double overall_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - time_start_learning).count();
            LOG(INFO) << "=========================================================================";
            LOG(INFO) << "Experiment finished" << std::endl;
            LOG(INFO) << "It took " << overall_time * 1e-3 << " s. That's " << overall_time * 1e-3 / 60 << " minutes.";
        }

        // Reads a trajectory file from disk and generates the residual observations.
        // Helpful when restarting a learning process that uses a residual model.
        void populate_observations(std::string filename, ParamVec params) {
            // Execute best policy so far on robot
            rollout_result obs_new;
            RolloutInfo rollout_info;
            if (!process_real_execution_results(filename, obs_new, rollout_info)) {
                LOG(FATAL) << "Something went wrong when reading from: " << filename;
            }

            // Calculate reward
            std::vector<Eigen::VectorXd> states;
            std::vector<Eigen::VectorXd> commands;
            for (const auto& e : obs_new) {
                states.push_back(std::get<0>(e));
                states.push_back(std::get<1>(e));
            }
            reward::RewardResultMap rewards = reward::calculate_rewards<Reward, RolloutInfo, Params>(*_reward, rollout_info, states, commands);
            LOG(INFO) << reward::rewards_to_stream(rewards).rdbuf();

            rollout_result obs_sim = _robot.template execute_with_sequence<Model>(obs_new, _policy.params(), false, _model);
            rollout_result obs_diff = calculate_diff_sim_real(obs_sim, obs_new);
            for (size_t i = 0; i < obs_diff.size(); i++) {
                Eigen::VectorXd state_vec = std::get<2>(obs_diff[i]);
                std::get<2>(obs_diff[i]) = state_vec;
            }
            // Check if it is better than the previous best if we are in a single-objective setting
            if (utils::getKeyCount(rewards) == 1) {
                double r_new {0.};
                const auto first_obj = rewards.cbegin();
                const auto first_obj_rewards = rewards.equal_range(first_obj->first);
                for (auto i = first_obj_rewards.first; i != first_obj_rewards.second; ++i) {
                    r_new += i->second.total_reward();
                }
                if (r_new > _best) {
                    _best = r_new;
                    _params_starting = params;
                }
            }

            // Append recorded difference data
            _observations.insert(_observations.end(), obs_diff.begin(), obs_diff.end());
        }

        void set_policy_params(ParamVec params) {
            _policy.set_params(params);
        }

        void set_reward_functions(std::shared_ptr<Reward> r) {
            _reward = r;
        }

        template <typename scene_function_t>
        void set_scene_setup(scene_function_t s) {
            _robot.set_scene_setup(s);
        }

        // Run an episode with a given set of parameters and a given model.
        // Saves the recorded data to given locations.
        reward::RewardResultMap play_iteration(const std::string& model_location, const std::string& param_location, const std::string& traj_sparse_filename = "/tmp/trajectory_sparse.dat",
                              const std::string& traj_dense_filename = "/tmp/trajectory_dense.dat", const std::string& traj_dense_ee_filename = "/tmp/trajectory_dense_ee.dat", const std::string& residual_filename = "/tmp/residual.dat") {
            ParamVec params = parameters::paramVecFromJson(parameters::fileToJson(param_location));
            LOG(INFO) << "Parameters loaded:\n" << parameters::paramVecToStream(params, false).rdbuf();
            _policy.set_params(params);

            if (!model_location.empty()) {
                LOG(INFO) << "Loading a model from " << model_location;
                _model.load_model(model_location);
            } else {
                LOG(INFO) << "Loading no model.";
            }
            reward::RewardResultMap reward;
            Policy p = _policy;
            p.set_params(_policy.params());
            _robot.execute_with_model(p, _model, *_reward, Params::skireil::T(), reward, true, false);
            // statistics for trajectories
            utils::save_traj_to_file(traj_sparse_filename, _robot.get_last_states(), _robot.get_last_commands());
            utils::save_traj_to_file(traj_dense_filename, _robot.get_last_dense_states(), _robot.get_last_dense_commands());
            utils::save_traj_to_file(traj_dense_ee_filename, _robot.get_last_dense_ee_pos(), _robot.get_last_dense_ee_rot());
            utils::save_traj_to_file(residual_filename, _robot.get_last_applied_residual_mean(), _robot.get_last_applied_residual_variance());
            return reward;
        }

        // Play a given trajectory and querying a given model.
        // This can be helpful for debugging to see what the model returns at given points. Observe that the policy parameters are not needed.
        void replay_trajectory_with_model(const std::string& model_location, const std::string& traj_filename, const std::string& traj_sparse_filename = "/tmp/trajectory_sparse.dat",
                              const std::string& traj_dense_filename = "/tmp/trajectory_dense.dat", const std::string& traj_dense_ee_filename = "/tmp/trajectory_dense_ee.dat", const std::string& residual_filename = "/tmp/residual.dat") {
            if (!model_location.empty()) {
                LOG(INFO) << "Loading a model from " << model_location;
                _model.load_model(model_location);
            } else {
                LOG(INFO) << "Loading no model.";
            }
            // Execute best policy so far on robot
            rollout_result obs_new;
            RolloutInfo rollout_info;
            if (!process_real_execution_results(traj_filename, obs_new, rollout_info)) {
                LOG(FATAL) << "Something went wrong when reading from: " << traj_filename;
            }
            // Calculate reward
            std::vector<Eigen::VectorXd> states;
            std::vector<Eigen::VectorXd> commands;
            for (const auto& e : obs_new) {
                states.push_back(std::get<0>(e));
                states.push_back(std::get<1>(e));
            }
            reward::RewardResultMap rewards = reward::calculate_rewards<Reward, RolloutInfo, Params>(*_reward, rollout_info, states, commands);
            LOG(INFO) << reward::rewards_to_stream(rewards).rdbuf();

            rollout_result obs_sim = _robot.execute_with_sequence(obs_new, _policy.params(), true, _model, residual_filename, true);
            // rollout_result obs_diff = calculate_diff_sim_real(obs_sim, obs_new);
        }

        PolicyOptimizer& policy_optimizer() { return _policy_optimizer; }
        const PolicyOptimizer& policy_optimizer() const { return _policy_optimizer; }

    protected:
        Robot _robot;
        Policy _policy{Policy(false)};
        Model _model;
        std::shared_ptr<Reward> _reward;
        PolicyOptimizer _policy_optimizer;
        std::ofstream _ofs_real, _ofs_esti, _ofs_traj_real, _ofs_traj_dummy, _ofs_results, _ofs_exp, _ofs_opt, _ofs_model;
        std::string _ofs_real_name, _ofs_real_ee_name, _ofs_opt_ee_name;

        ParamVec _params_starting;
        double _best;
        bool _random_policies;
        unsigned int _opt_iters, _model_evals;
        int _iteration;
        double _max_reward;
        ParamVec _max_params;
        double _boundary;
        std::mutex _iter_mutex;
        std::vector<EvalResult> _best_evals;
        std::vector<EvalResult> _all_evals;

        // state, action, prediction
        std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>> _observations;

        // Actual optimize function that would be called by the optimizer
        reward::RewardResultMap _optimize_policy(const ParamVec& params, bool eval_grad = false)
        {
            auto start = std::chrono::high_resolution_clock::now();
            Policy policy(false);
            policy.set_params(params);

            // Do N evaluations in a parallel loop
            int N = (Params::skireil::stochastic_evaluation()) ? Params::skireil::opt_evals() : 1;
            CHECK_GE(N, 1) << "A policy needs to be evaluated at least once.";
            std::vector<reward::RewardResultMap> rews;
            rews.resize(N);
            limbo::tools::par::loop(0, N, [&](size_t i) {
                // Policy objects are not thread-safe usually
                Policy p(false);
                p.set_params(policy.params());

                reward::RewardResultMap R;
                if (Params::skireil::learn_real_system()) {
                    _robot.execute_with_real_system(p, *_reward, Params::skireil::T(), R);
                } else {
                    _robot.execute_with_model(p, _model, *_reward, Params::skireil::T(), R, false, false);
                }

                rews.at(i) = R;
            });
            if (Params::meta_conf::verbose()) {
                auto finish = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = finish - start;
                LOG(INFO) << "Execute function took " << elapsed.count() << " s\n" << std::flush;
            }
            reward::RewardResultMap acc_rew = reward::reward_result_map_from_vec(rews);
            double r {0.};
            if (utils::getKeyCount(acc_rew) == 1) {
                r = reward::get_reward_by_objective(acc_rew, acc_rew.cbegin()->first);
            }

            // Save results and print progress
            _iter_mutex.lock();
            _opt_iters++;
            _model_evals += N;
            LOG_IF(INFO, _opt_iters % 100 == 0) << "Finished " << _opt_iters << " iterations. Did " << _model_evals << " model evaluations.";
            _all_evals.push_back({r, _opt_iters, policy.params()});
            if (_max_reward < r) {
                _max_reward = r;
                _max_params = policy.params();
                _best_evals.push_back({r, _opt_iters, policy.params()});
                LOG(INFO) << _best_evals.back().pretty_print();
            } else if (r > (0.75*_max_reward + 0.01)) {
                LOG(INFO) << _all_evals.back().pretty_print(false, false, true);
            } else {
                if (Params::meta_conf::verbose()) {
                    LOG(INFO) << _all_evals.back().pretty_print(false, true);
                }
            }
            _iter_mutex.unlock();
            return acc_rew;
        }
    }; // namespace skireil
} // namespace skireil

#endif
