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
#ifndef SKIREIL_REWARD_REWARD_FUNCTIONS_HPP
#define SKIREIL_REWARD_REWARD_FUNCTIONS_HPP

#include <skireil/reward/reward.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <cmath>

#include <glog/logging.h>

namespace skireil {
    namespace reward {

        Eigen::VectorXd get_ee_pos(std::shared_ptr<robot_dart::Robot> robot, const Eigen::VectorXd& j_pos, const std::vector<std::string>& dof_names, const std::string& link_name) {
            robot->fix_to_world();
            robot->set_position_enforced(true);
            robot->set_positions(j_pos, dof_names);
            auto bd = robot->skeleton()->getBodyNode(link_name);
            Eigen::VectorXd ee_pos = bd->getTransform().translation();
            return ee_pos;
        }

        template <typename RolloutInfo>
        struct FixedSuccessReward : public skireil::reward::Reward<RolloutInfo> {
            bool configure(const nlohmann::json& conf, std::shared_ptr<robot_dart::Robot> robot, const std::vector<std::string>& robot_dof) {
                if (!conf.contains("value")) {
                    LOG(FATAL) << "Reward 'FixedSuccessReward': 'value' is not specified. Aborting.";
                } else {
                    _value = conf["value"].get<double>();
                }
                if (conf.contains("per_time_step")) {
                    _per_time_step = conf["per_time_step"].get<bool>();
                }
                return true;
            }

            std::pair<double, bool> operator()(RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state) const
            {
                bool finished = info.finish_time >= 0.0 ? true : false;
                double reward = 0.0;
                if (finished) {
                    reward = _value;
                }
                if (_per_time_step) {
                    return std::make_pair(reward, finished);
                } else {
                    return std::make_pair(reward/(info.T/info.dt), finished);
                }
            }

            private:
                double _value{0.0};
                bool _per_time_step{false};
        };

        template <typename RolloutInfo>
        struct GoalDistanceTranslationReward : public skireil::reward::Reward<RolloutInfo> {
            bool configure(const nlohmann::json& conf, std::shared_ptr<robot_dart::Robot> robot, const std::vector<std::string>& robot_dof) {
                _simulated_robot = CHECK_NOTNULL(robot)->clone();
                _robot_dof = robot_dof;
                if (!conf.contains("link_name")) {
                    LOG(FATAL) << "Reward 'GoalDistanceReward': 'link_name' is not specified. Aborting.";
                } else {
                    _link_name = conf["link_name"].get<std::string>();
                    if (_simulated_robot->skeleton()->getBodyNode(_link_name) == NULL) {
                        throw std::invalid_argument("Link name '" + _link_name + "' does not appear in robot.");
                    }
                }
                if (!conf.contains("width")) {
                    LOG(FATAL) << "Reward 'GoalDistanceReward': 'width' is not specified. Aborting.";
                } else {
                    _width_sq = conf["width"].get<double>();
                    _width_sq = _width_sq * _width_sq;
                }
                if (!conf.contains("min_dist")) {
                    LOG(FATAL) << "Reward 'GoalDistanceReward': 'min_dist' is not specified. Aborting.";
                } else {
                    _min_dist = conf["min_dist"].get<double>();
                }
                if (!conf.contains("goal")) {
                    LOG(FATAL) << "Reward 'GoalDistanceReward': 'goal' is not specified. Aborting.";
                } else {
                    if (!conf["goal"].contains("x") || !conf["goal"].contains("y") || !conf["goal"].contains("z")) {
                        LOG(FATAL) << "Reward 'GoalDistanceReward': Goal coordinates, x,y,z are missing. Aborting.";
                    }
                    _goal[0] = conf["goal"]["x"].get<double>();
                    _goal[1] = conf["goal"]["y"].get<double>();
                    _goal[2] = conf["goal"]["z"].get<double>();
                }
                if (conf.contains("negative")) {
                    if (conf["negative"].get<bool>()) {
                        _sign = -1.0;
                    }
                }
                return true;
            }

            std::pair<double, bool> operator()(RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state) const
            {
                Eigen::VectorXd ee_pos = get_ee_pos(_simulated_robot, to_state.head(7), _robot_dof, _link_name);
                double dee = (ee_pos - _goal).squaredNorm();
                // When changing this, remember to update the print function.
                double reward = _sign * std::exp(-0.5 / _width_sq * (dee+_min_dist));
                return std::make_pair(reward, true);
            }

            private:
                double _sign{1.0};            
                std::string _link_name;
                double _width_sq{0.0};
                double _min_dist{0.0};
                Eigen::VectorXd _goal{Eigen::VectorXd(3)};
                std::shared_ptr<robot_dart::Robot> _simulated_robot;
                std::vector<std::string> _robot_dof;
        };

        template <typename RolloutInfo>
        struct EndEffectorReferencePosition : public skireil::reward::Reward<RolloutInfo> {
            bool configure(const nlohmann::json& conf, std::shared_ptr<robot_dart::Robot> robot, const std::vector<std::string>& robot_dof) {
                _simulated_robot = CHECK_NOTNULL(robot)->clone();
                _robot_dof = robot_dof;
                if (!conf.contains("link_name")) {
                    LOG(FATAL) << "Reward 'EndEffectorReferencePosition': 'link_name' is not specified. Aborting.";
                } else {
                    _link_name = conf["link_name"].get<std::string>();
                    if (_simulated_robot->skeleton()->getBodyNode(_link_name) == NULL) {
                        throw std::invalid_argument("Link name '" + _link_name + "' does not appear in robot.");
                    }
                }
                if (!conf.contains("width")) {
                    LOG(FATAL) << "Reward 'EndEffectorReferencePosition': 'width' is not specified. Aborting.";
                } else {
                    _width_sq = conf["width"].get<double>();
                    _width_sq = _width_sq * _width_sq;
                }
                if (!conf.contains("min_dist")) {
                    LOG(FATAL) << "Reward 'EndEffectorReferencePosition': 'min_dist' is not specified. Aborting.";
                } else {
                    _min_dist = conf["min_dist"].get<double>();
                }
                if (conf.contains("negative")) {
                    if (conf["negative"].get<bool>()) {
                        _sign = -1.0;
                    }
                }
                return true;
            }

            std::pair<double, bool> operator()(RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state) const
            {
                Eigen::VectorXd ee_pos = get_ee_pos(_simulated_robot, to_state.head(7), _robot_dof, _link_name);
                Eigen::VectorXd ref_pos = action.head(3);
                double dee = (ee_pos - ref_pos).squaredNorm();
                double reward = _sign * std::exp(-0.5 / _width_sq * (dee+_min_dist));
                return std::make_pair(reward, true);
            }

            private:
                double _sign{1.0};            
                std::string _link_name;
                double _width_sq{0.0};
                double _min_dist{0.0};
                std::shared_ptr<robot_dart::Robot> _simulated_robot;
                std::vector<std::string> _robot_dof;
        };


        template <typename RolloutInfo>
        struct BoxAvoidanceReward : public skireil::reward::Reward<RolloutInfo> {
            typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point_t;
            typedef boost::geometry::model::box<point_t> box_t;

            bool configure(const nlohmann::json& conf, std::shared_ptr<robot_dart::Robot> robot, const std::vector<std::string>& robot_dof) {
                _simulated_robot = CHECK_NOTNULL(robot)->clone();
                _robot_dof = robot_dof;
                if (!conf.contains("link_name")) {
                    LOG(FATAL) << "Reward 'BoxAvoidanceReward': 'link_name' is not specified. Aborting.";
                } else {
                    _link_name = conf["link_name"].get<std::string>();
                    if (_simulated_robot->skeleton()->getBodyNode(_link_name) == NULL) {
                        throw std::invalid_argument("Link name '" + _link_name + "' does not appear in robot.");
                    }
                }
                if (!conf.contains("width")) {
                    LOG(FATAL) << "Reward 'BoxAvoidanceReward': 'width' is not specified. Aborting.";
                } else {
                    _width = conf["width"].get<double>();
                }
                if (!conf.contains("min_dist")) {
                    LOG(FATAL) << "Reward 'BoxAvoidanceReward': 'min_dist' is not specified. Aborting.";
                } else {
                    _min_dist = conf["min_dist"].get<double>();
                }
                if (!conf.contains("box")) {
                    LOG(FATAL) << "Reward 'BoxAvoidanceReward': 'box' is not specified. Aborting.";
                } else {
                    if (!conf["box"].contains("x_min") || !conf["box"].contains("y_min") || !conf["box"].contains("z_min") || !conf["box"].contains("x_max") || !conf["box"].contains("y_max") || !conf["box"].contains("z_max")) {
                        LOG(FATAL) << "Reward 'BoxAvoidanceReward': Box min and max coordinates, x,y,z are missing. Aborting.";
                    }
                    point_t min_corner {conf["box"]["x_min"].get<double>(), conf["box"]["y_min"].get<double>(), conf["box"]["z_min"].get<double>()};
                    point_t max_corner {conf["box"]["x_max"].get<double>(), conf["box"]["y_max"].get<double>(), conf["box"]["z_max"].get<double>()};
                    _box = box_t(min_corner, max_corner);
                }
                if (conf.contains("negative")) {
                    if (conf["negative"].get<bool>()) {
                        _sign = -1.0;
                    }
                }
                return true;
            }

            std::pair<double, bool> operator()(RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state) const
            {
                Eigen::VectorXd ee_position = get_ee_pos(_simulated_robot, to_state.head(7), _robot_dof, _link_name);
                point_t ee_pos{ee_position[0], ee_position[1], ee_position[2]};
                double distance = boost::geometry::distance(ee_pos, _box);
                if (distance < 0) {
                    distance = 0.0;
                }
                double reward = _sign/(_width*std::pow(distance + _min_dist, 2.0));
                if (distance < _min_dist) {
                    return std::make_pair(reward, false);    
                } else {
                    return std::make_pair(reward, true);
                }
            }

            private:
                double _sign{1.0};
                box_t _box;
                std::string _link_name;
                double _width{0.0};
                double _min_dist{0.0};
                Eigen::VectorXd _goal{Eigen::VectorXd(3)};
                std::shared_ptr<robot_dart::Robot> _simulated_robot;
                std::vector<std::string> _robot_dof;
        };

        template <typename RolloutInfo>
        struct LinearDistanceToBoxReward : public skireil::reward::Reward<RolloutInfo> {
            typedef boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian> point_t;
            typedef boost::geometry::model::box<point_t> box_t;

            bool configure(const nlohmann::json& conf, std::shared_ptr<robot_dart::Robot> robot, const std::vector<std::string>& robot_dof) {
                _simulated_robot = CHECK_NOTNULL(robot)->clone();
                _robot_dof = robot_dof;
                if (!conf.contains("link_name")) {
                    LOG(FATAL) << "Reward 'LinearDistanceToBoxReward': 'link_name' is not specified. Aborting.";
                } else {
                    _link_name = conf["link_name"].get<std::string>();
                    if (_simulated_robot->skeleton()->getBodyNode(_link_name) == NULL) {
                        throw std::invalid_argument("Link name '" + _link_name + "' does not appear in robot.");
                    }
                }
                if (!conf.contains("width")) {
                    LOG(FATAL) << "Reward 'LinearDistanceToBoxReward': 'width' is not specified. Aborting.";
                } else {
                    _width = conf["width"].get<double>();
                }
                if (!conf.contains("min_dist")) {
                    LOG(FATAL) << "Reward 'LinearDistanceToBoxReward': 'min_dist' is not specified. Aborting.";
                } else {
                    _min_dist = conf["min_dist"].get<double>();
                }
                if (!conf.contains("box")) {
                    LOG(FATAL) << "Reward 'LinearDistanceToBoxReward': 'box' is not specified. Aborting.";
                } else {
                    if (!conf["box"].contains("x_min") || !conf["box"].contains("y_min") || !conf["box"].contains("z_min") || !conf["box"].contains("x_max") || !conf["box"].contains("y_max") || !conf["box"].contains("z_max")) {
                        LOG(FATAL) << "Reward 'LinearDistanceToBoxReward': Box min and max coordinates, x,y,z are missing. Aborting.";
                    }
                    point_t min_corner {conf["box"]["x_min"].get<double>(), conf["box"]["y_min"].get<double>(), conf["box"]["z_min"].get<double>()};
                    point_t max_corner {conf["box"]["x_max"].get<double>(), conf["box"]["y_max"].get<double>(), conf["box"]["z_max"].get<double>()};
                    _box = box_t(min_corner, max_corner);
                }
                if (conf.contains("negative")) {
                    if (conf["negative"].get<bool>()) {
                        _sign = -1.0;
                    }
                }
                return true;
            }

            std::pair<double, bool> operator()(RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state) const
            {
                Eigen::VectorXd ee_position = get_ee_pos(_simulated_robot, to_state.head(7), _robot_dof, _link_name);
                point_t ee_pos{ee_position[0], ee_position[1], ee_position[2]};
                double distance = boost::geometry::distance(ee_pos, _box);
                if (distance < 0) {
                    distance = 0.0;
                }
                double reward = _sign/(_width*(distance + _min_dist));
                if (distance < _min_dist) {
                    return std::make_pair(reward, false);    
                } else {
                    return std::make_pair(reward, true);
                }
            }

            private:
                double _sign{1.0};
                box_t _box;
                std::string _link_name;
                double _width{0.0};
                double _min_dist{0.0};
                Eigen::VectorXd _goal{Eigen::VectorXd(3)};
                std::shared_ptr<robot_dart::Robot> _simulated_robot;
                std::vector<std::string> _robot_dof;
        };

        template <typename RolloutInfo>
        struct ForceApplicationReward : public skireil::reward::Reward<RolloutInfo> {
            bool configure(const nlohmann::json& conf, std::shared_ptr<robot_dart::Robot> robot, const std::vector<std::string>& robot_dof) {
                if (conf.contains("negative")) {
                    if (conf["negative"].get<bool>()) {
                        _sign = -1.0;
                    }
                }
                return true;
            }

            std::pair<double, bool> operator()(RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state) const
            {
                double reward = 0.0;
                double _action_val = std::abs(action(7)) + std::abs(action(8)) + std::abs(action(9));
                reward = _sign * _action_val; 
                return std::make_pair(reward, true);
            }

            private:
                double _sign{1.0}; 
        };


        template <typename RolloutInfo>
        struct ObjectPoseReward : public skireil::reward::Reward<RolloutInfo> {
            bool configure(const nlohmann::json& conf, std::shared_ptr<robot_dart::Robot> robot, const std::vector<std::string>& robot_dof) {
                if (!conf.contains("width")) {
                    LOG(FATAL) << "Reward 'ObjectPoseReward': 'width' is not specified. Aborting.";
                } else {
                    _width_sq = conf["width"].get<double>();
                    _width_sq = _width_sq * _width_sq;
                }
                if (!conf.contains("min_dist")) {
                    LOG(FATAL) << "Reward 'ObjectPoseReward': 'min_dist' is not specified. Aborting.";
                } else {
                    _min_dist = conf["min_dist"].get<double>();
                }
                if (!conf.contains("start_index")) {
                    LOG(FATAL) << "Reward 'ObjectPoseReward': 'start_index' is not specified. Aborting.";
                } else {
                    _start_index = conf["start_index"].get<double>();
                }
                if (!conf.contains("goal") || !conf["goal"].contains("position")) {
                    LOG(FATAL) << "Reward 'ObjectPoseReward': 'goal' is not specified. Aborting.";
                } else {
                    if (!conf["goal"].contains("position")) {
                        LOG(FATAL) << "Reward 'ObjectPoseReward': 'goal: position' is not specified. Aborting.";
                    } else {
                        if (!conf["goal"]["position"].contains("x") || !conf["goal"]["position"].contains("y") || !conf["goal"]["position"].contains("z")) {
                            LOG(FATAL) << "Reward 'ObjectPoseReward': Goal coordinates, x,y,z are missing. Aborting.";
                        }
                        _pos[0] = conf["goal"]["position"]["x"].get<double>();
                        _pos[1] = conf["goal"]["position"]["y"].get<double>();
                        _pos[2] = conf["goal"]["position"]["z"].get<double>();
                    }
                    if (!conf["goal"].contains("orientation")) {
                        LOG(FATAL) << "Reward 'ObjectPoseReward': 'goal: orientation' is not specified. Aborting.";
                    } else {
                    if (!conf["goal"]["orientation"].contains("x") || !conf["goal"]["orientation"].contains("y") || !conf["goal"]["orientation"].contains("z") || !conf["goal"]["orientation"].contains("w")) {
                            LOG(FATAL) << "Reward 'ObjectPoseReward': Goal coordinates, x,y,z are missing. Aborting.";
                        }
                        _rot.x() = conf["goal"]["orientation"]["x"].get<double>();
                        _rot.y() = conf["goal"]["orientation"]["y"].get<double>();
                        _rot.z() = conf["goal"]["orientation"]["z"].get<double>();
                        _rot.w() = conf["goal"]["orientation"]["w"].get<double>();
                    }
                }
                if (conf.contains("negative")) {
                    if (conf["negative"].get<bool>()) {
                        _sign = -1.0;
                    }
                }
                return true;
            }

            std::pair<double, bool> operator()(RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state) const
            {
                LOG_IF(FATAL, from_state.size() < _start_index + 7) << "State vector is too small. Expected " << _start_index + 7 << " got " << from_state.size();
                Eigen::VectorXd current_pos(3);
                current_pos << to_state.segment(_start_index, 3);
                Eigen::Vector4d coeffs = to_state.segment(_start_index + 3, 4);
                // Quaterniond are initialized w,x,y,z, but output coeffs x,y,z,w
                Eigen::Quaterniond current_rot(coeffs[3], coeffs[0], coeffs[1], coeffs[2]);

                // Distances
                double d_trans = (current_pos - _pos).norm();
                double d_rot = _rot.angularDistance(current_rot);
                double error = d_trans + _rot_multi * d_rot;
                double reward = _sign * std::exp(-0.5 / _width_sq * (error + _min_dist));
                return std::make_pair(reward, true);
            }

            private:
                double _sign{1.0};
                double _width_sq{0.0};
                double _min_dist{0.0};
                int _start_index{0};
                // 20°, 0.035rad rotation is roughly as bad as 0.01m distance.
                double _rot_multi{0.3};
                Eigen::VectorXd _pos{Eigen::VectorXd(3)};
                Eigen::Quaterniond _rot{Eigen::Quaterniond::Identity()};
        };

        template <typename RolloutInfo>
        struct ObjectPositionReward : public skireil::reward::Reward<RolloutInfo> {
            bool configure(const nlohmann::json& conf, std::shared_ptr<robot_dart::Robot> robot, const std::vector<std::string>& robot_dof) {
                if (!conf.contains("width")) {
                    LOG(FATAL) << "Reward 'ObjectPositionReward': 'width' is not specified. Aborting.";
                } else {
                    _width_sq = conf["width"].get<double>();
                    _width_sq = _width_sq * _width_sq;
                }
                if (!conf.contains("min_dist")) {
                    LOG(FATAL) << "Reward 'ObjectPositionReward': 'min_dist' is not specified. Aborting.";
                } else {
                    _min_dist = conf["min_dist"].get<double>();
                }
                if (!conf.contains("start_index")) {
                    LOG(FATAL) << "Reward 'ObjectPositionReward': 'start_index' is not specified. Aborting.";
                } else {
                    _start_index = conf["start_index"].get<double>();
                }
                if (!conf.contains("goal") || !conf["goal"].contains("position")) {
                    LOG(FATAL) << "Reward 'ObjectPositionReward': 'goal' is not specified. Aborting.";
                } else {
                    if (!conf["goal"].contains("position")) {
                        LOG(FATAL) << "Reward 'ObjectPositionReward': 'goal: position' is not specified. Aborting.";
                    } else {
                        if (!conf["goal"]["position"].contains("x") || !conf["goal"]["position"].contains("y") || !conf["goal"]["position"].contains("z")) {
                            LOG(FATAL) << "Reward 'ObjectPositionReward': Goal coordinates, x,y,z are missing. Aborting.";
                        }
                        _pos[0] = conf["goal"]["position"]["x"].get<double>();
                        _pos[1] = conf["goal"]["position"]["y"].get<double>();
                        _pos[2] = conf["goal"]["position"]["z"].get<double>();
                    }
                }
                if (conf.contains("negative")) {
                    if (conf["negative"].get<bool>()) {
                        _sign = -1.0;
                    }
                }
                return true;
            }

            std::pair<double, bool> operator()(RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state) const
            {
                LOG_IF(FATAL, from_state.size() < _start_index + 7) << "State vector is too small. Expected " << _start_index + 7 << " got " << from_state.size();
                Eigen::VectorXd current_pos(3);
                current_pos << to_state.segment(_start_index, 3);

                // Distances
                double d_trans = (current_pos - _pos).norm();
                double reward = _sign * std::exp(-0.5 / _width_sq * (d_trans + _min_dist));
                return std::make_pair(reward, true);
            }

            private:
                double _sign{1.0};
                double _width_sq{0.0};
                double _min_dist{0.0};
                int _start_index{0};
                Eigen::VectorXd _pos{Eigen::VectorXd(3)};
        };

        template <typename RolloutInfo>
        struct ObjectOrientationReward : public skireil::reward::Reward<RolloutInfo> {
            bool configure(const nlohmann::json& conf, std::shared_ptr<robot_dart::Robot> robot, const std::vector<std::string>& robot_dof) {
                if (!conf.contains("width")) {
                    LOG(FATAL) << "Reward 'ObjectOrientationReward': 'width' is not specified. Aborting.";
                } else {
                    _width_sq = conf["width"].get<double>();
                    _width_sq = _width_sq * _width_sq;
                }
                if (!conf.contains("min_dist")) {
                    LOG(FATAL) << "Reward 'ObjectOrientationReward': 'min_dist' is not specified. Aborting.";
                } else {
                    _min_dist = conf["min_dist"].get<double>();
                }
                if (!conf.contains("start_index")) {
                    LOG(FATAL) << "Reward 'ObjectOrientationReward': 'start_index' is not specified. Aborting.";
                } else {
                    _start_index = conf["start_index"].get<double>();
                }
                if (!conf.contains("goal") || !conf["goal"].contains("orientation")) {
                    LOG(FATAL) << "Reward 'ObjectOrientationReward': 'goal' is not specified. Aborting.";
                } else {
                    if (!conf["goal"].contains("orientation")) {
                        LOG(FATAL) << "Reward 'ObjectOrientationReward': 'goal: orientation' is not specified. Aborting.";
                    } else {
                    if (!conf["goal"]["orientation"].contains("x") || !conf["goal"]["orientation"].contains("y") || !conf["goal"]["orientation"].contains("z") || !conf["goal"]["orientation"].contains("w")) {
                            LOG(FATAL) << "Reward 'ObjectOrientationReward': Goal coordinates, x,y,z are missing. Aborting.";
                        }
                        _rot.x() = conf["goal"]["orientation"]["x"].get<double>();
                        _rot.y() = conf["goal"]["orientation"]["y"].get<double>();
                        _rot.z() = conf["goal"]["orientation"]["z"].get<double>();
                        _rot.w() = conf["goal"]["orientation"]["w"].get<double>();
                    }
                }
                if (conf.contains("negative")) {
                    if (conf["negative"].get<bool>()) {
                        _sign = -1.0;
                    }
                }
                return true;
            }

            std::pair<double, bool> operator()(RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state) const
            {
                LOG_IF(FATAL, from_state.size() < _start_index + 7) << "State vector is too small. Expected " << _start_index + 7 << " got " << from_state.size();
                Eigen::Vector4d coeffs = to_state.segment(_start_index + 3, 4);
                // Quaterniond are initialized w,x,y,z, but output coeffs x,y,z,w
                Eigen::Quaterniond current_rot(coeffs[3], coeffs[0], coeffs[1], coeffs[2]);

                // Distances
                double d_rot = _rot.angularDistance(current_rot);
                double reward = _sign * std::exp(-0.5 / _width_sq * (d_rot + _min_dist));
                return std::make_pair(reward, true);
            }

            private:
                double _sign{1.0};
                double _width_sq{0.0};
                double _min_dist{0.0};
                int _start_index{0};
                Eigen::Quaterniond _rot{Eigen::Quaterniond::Identity()};
        };
    } // namespace reward
} // namespace skireil

#endif