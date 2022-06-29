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
#ifndef SKIREIL_REWARD_REWARD_RESULT_HPP
#define SKIREIL_REWARD_REWARD_RESULT_HPP

#include <Eigen/Core>
#include <string>
#include <map>

namespace skireil {
    namespace reward {

        struct RewardResult {
            double weight{1.0};
            std::string objective;
            std::string name;
            std::string key;
            std::vector<double> rewards;
            bool feasibility{true};
            bool reward_per_timestep{true};

            void set_feasibility(bool f) {
                if (!f) {
                    feasibility = false;
                }
            }

            double get_acc_reward() const {
                if (reward_per_timestep) {
                    return total_reward();
                } else {
                    return mean_reward();
                }
            }

            std::vector<double> get_rewards() const {
                std::vector<double> r_vec = rewards;
                std::transform(r_vec.begin(), r_vec.end(), r_vec.begin(), [&](auto& c){return c*weight;});
                return r_vec;
            }

            double total_reward() const {
                if (reward_per_timestep) {
                    return weight * std::accumulate(rewards.begin(), rewards.end(), 0.0);
                } else {
                    return std::accumulate(rewards.begin(), rewards.end(), 0.0);
                }
            }

            double mean_reward() const {
                return total_reward() / rewards.size();
            }
        };

        using RewardResultMap = std::multimap<std::string, reward::RewardResult>;

        RewardResultMap reward_result_map_from_vec(std::vector<RewardResultMap>& vec) {
            size_t runs = vec.size();
            CHECK_GE(runs, 1) << "Need at least 1 RewardResult.";

            // Build up a map by key
            std::map<std::string, reward::RewardResult> res;
            for (const auto &el: vec.at(0)) {
                reward::RewardResult rr = el.second;
                double total_reward = rr.total_reward();
                rr.rewards.clear();
                rr.rewards.push_back(total_reward);
                rr.reward_per_timestep = false;
                auto ins = res.insert(std::pair<std::string, reward::RewardResult>(el.second.key, rr));
                CHECK_EQ(ins.second, true) << "Keys of reward functions need to be unique. '" << ins.first->first << "' exists already.";
            }
            // Add the other evaluations. This fails if we have a new unknown reward function
            for (size_t run = 1; run < runs; run++) {
                for (const auto &el: vec.at(run)) {
                    res.at(el.second.key).rewards.push_back(el.second.total_reward());
                }
            }
            RewardResultMap ret;
            for (const auto& r : res) {
                ret.insert(std::pair<std::string, reward::RewardResult>(r.second.objective, r.second));
            }
            return ret;
        }

        std::mutex reward_mutex;

        template <typename Reward, typename RolloutInfo, typename Params>
        RewardResultMap calculate_rewards(Reward& world, RolloutInfo& rollout_info, const std::vector<Eigen::VectorXd>& noiseless_states, const std::vector<Eigen::VectorXd>& commands) {
            // This lock is necessary since every reward function has only one robot object. If
            std::lock_guard<std::mutex> lock(reward_mutex);
            CHECK_EQ(noiseless_states.size(), commands.size()) << "Commands and noiseless states need to have the same size.";
            // Parallelized reward calculation per objective.
            // Results saved in pre-allocated vectors to avoid race conditions.
            std::vector<std::string> keys = utils::getKeys(world);
            std::vector<std::vector<reward::RewardResult>> rewards;
            rewards.resize(keys.size());
            for (size_t k = 0; k < keys.size(); k++) {
                size_t num_el = world.count(keys.at(k));
                std::vector<reward::RewardResult> obj_res;
                obj_res.resize(num_el);
                // Get a vector of reward functions
                std::vector<typename Reward::mapped_type> v;
                std::pair<typename Reward::iterator,typename Reward::iterator> aRange = world.equal_range(keys.at(k));
                // for (auto i = aRange.first; i != aRange.second; ++i) {
                //     v.push_back(i->second);
                // }
                std::transform(aRange.first, aRange.second, std::back_inserter(v), [](auto element){return element.second;});
                CHECK_EQ(num_el, v.size()) << ": Vector of reward functions must have the same size as ";

                // Evaluate reward functions
                for (size_t i = 0; i < num_el; i++) {
                    // To make it thread-safe
                    RolloutInfo r_i = rollout_info;
                    reward::RewardResult sr;
                    sr.objective = keys.at(k);
                    sr.name = v.at(i)->get_name();
                    sr.key = v.at(i)->get_key();
                    sr.weight = v.at(i)->get_weight();
                    for (size_t j = 0; j < noiseless_states.size() - 1; j++) {
                        r_i.t = Params::skireil::dt() *j;
                        auto r_f = v.at(i)->observe(r_i, noiseless_states[j], commands[j], noiseless_states[j + 1], false);
                        sr.rewards.push_back(r_f.first);
                        sr.set_feasibility(r_f.second);
                    }
                    obj_res.at(i) = sr;
                };
                rewards.at(k) = obj_res;
            };
            // Save result in a multimap with objective as key
            std::multimap<std::string, reward::RewardResult> res;
            for (const auto& r_c : rewards) {
                for (const auto& r : r_c) {
                    res.insert(std::pair<std::string, reward::RewardResult>(r.objective, r));
                }
            }
            return res;
        }

        double get_reward_by_objective(const RewardResultMap& rewards, std::string objective) {
            double r{0};
            const auto first_obj_rewards = rewards.equal_range(objective);
            for (auto i = first_obj_rewards.first; i != first_obj_rewards.second; ++i) {
                r += i->second.get_acc_reward();
            }
            return r;
        }

        std::stringstream rewards_to_stream(const RewardResultMap& rewards, bool verbose = true, bool one_line = false) {
            std::stringstream s;
            std::vector<std::string> objs = utils::getKeys(rewards);
            std::string endl;
            if (!one_line) {
                endl = "\n";
            }
            for (const auto& obj : objs) {
                s << "'" << obj << "' total: " << get_reward_by_objective(rewards, obj) << endl;
                if (verbose) {
                    const auto rf = rewards.equal_range(obj);
                    for (auto i = rf.first; i != rf.second; ++i) {
                        s << "\t" << "'" << i->second.name << "', '" << i->second.key << "': " << i->second.get_acc_reward() << endl;
                    }
                }
            }
            return s;
        }

        bool check_feasibility(const RewardResultMap& rewards) {
            std::vector<std::string> objs = utils::getKeys(rewards);
            for (const auto& obj : objs) {
                const auto first_obj_rewards = rewards.equal_range(obj);
                for (auto i = first_obj_rewards.first; i != first_obj_rewards.second; ++i) {
                    if (!i->second.feasibility) {
                        return false;
                    }
                }
            }
            return true;
        }

    } // namespace reward
} // namespace skireil

#endif