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
#ifndef SKIREIL_POLICY_BT_POLICY_HPP
#define SKIREIL_POLICY_BT_POLICY_HPP

#include <Eigen/Core>
#include <limbo/tools/random_generator.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/JointState.h>
#include <cartesian_trajectory_generator/OverlayMotionConf.h>
#include <skireil/ros_msgs/SkirosNextAction.h>
#include <skireil/ParamFloat.h>
#include <skireil/ParamInt.h>
#include <skireil/ParamString.h>
#include <skireil/ros_msgs/ManageSkirosWorker.h>
#include <skireil/controller/motion_generator_configuration.hpp>
#include <glog/logging.h>
#include <robot_dart/robot.hpp>
#include <chrono>


namespace skireil {
    namespace policy {
        using ParamVec = std::vector<std::shared_ptr<skireil::parameters::ParamBase>>;

        template <typename Params>
        struct SkiROSPolicy {
        public:
            SkiROSPolicy(bool init_connection = true)
            {
                auto start = std::chrono::high_resolution_clock::now();

                if (!init_connection){
                    _has_connection = false;
                    return;
                }
                // Get a worker assigned.
                _ac_worker_manager = _node_handle.serviceClient<skireil::ManageSkirosWorker>(Params::bt_policy::worker_manager());
                // TODO: This waitForExistence doesn't seem to work reliably when the service appears later.
                if (!_ac_worker_manager.waitForExistence(ros::Duration(0.1))) {
                    LOG(ERROR) << "Could not reach service at " << Params::bt_policy::worker_manager() << ". Waiting forever." << std::flush;
                    _ac_worker_manager.waitForExistence();
                }
                auto finish = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double>  elapsed = finish - start;
                if (elapsed.count() > 0.1) {
                    LOG(INFO) << "Elapsed time to contact worker manager: " << elapsed.count() << " s\n" << std::flush;
                }
                start = std::chrono::high_resolution_clock::now();

                skireil::ManageSkirosWorker worker;
                worker.request.request = true;
                while (!_ac_worker_manager || ! _ac_worker_manager.call(worker)) {
                    LOG(ERROR) << "Worker manager service call failed. Will retry forever." << std::flush;
                    _ac_worker_manager = _node_handle.serviceClient<skireil::ManageSkirosWorker>(Params::bt_policy::worker_manager());
                    ros::Duration(0.05).sleep();
                }
                _worker_number = worker.response.worker_number;
                finish = std::chrono::high_resolution_clock::now();
                elapsed = finish - start;
                if (elapsed.count() > 0.1) {
                    LOG(INFO) << _worker_number << ": Elapsed time to get a worker: " << elapsed.count() << " s\n" << std::flush;
                }
                start = std::chrono::high_resolution_clock::now();
                
                // Build up a connection to that worker.
                _worker_topic = Params::bt_policy::next_action_service();
                _worker_topic += std::to_string(_worker_number);
                _ac_next_action = _node_handle.serviceClient<skireil::SkirosNextAction>(_worker_topic, true);

                if (!_ac_next_action.waitForExistence(ros::Duration(0.1))) {
                    LOG(ERROR) << "Could not reach service at " << _worker_topic << ". Waiting forever." << std::flush;
                    _ac_next_action.waitForExistence();
                }
                finish = std::chrono::high_resolution_clock::now();
                elapsed = finish - start;
                if (elapsed.count() > 0.1) {
                    LOG(INFO) << _worker_number << ": Elapsed time to connect to worker: " << elapsed.count() << " s\n" << std::flush;
                }
            }

            ~SkiROSPolicy(){
                if (Params::bt_policy::verbose()) {
                    LOG(INFO) << _worker_number << ": Policy destructor." << std::flush;
                }
                if (!_has_connection) {
                    return;
                }
                // Stop the task in Skiros if it's still running
                skireil::SkirosNextAction srv_msg;
                srv_msg.request.type = srv_msg.request.STOP_TASK;
                srv_msg.request.sim_time = 1000;
                call_service(srv_msg);
                if (Params::bt_policy::verbose()) {
                    LOG(INFO) << _worker_number << ": Successfully ended task." << std::flush;
                }
                // Release the worker when we're done
                skireil::ManageSkirosWorker worker;
                worker.request.release = _worker_number;
                while (!_ac_worker_manager || !_ac_worker_manager.call(worker)) {
                    LOG_EVERY_N(ERROR, 200) << _worker_number << ": Worker service call failed. Will retry forever." << std::flush;
                    _ac_worker_manager = _node_handle.serviceClient<skireil::ManageSkirosWorker>(Params::bt_policy::worker_manager());
                    ros::Duration(0.05).sleep();
                }
                if (Params::bt_policy::verbose()) {
                    LOG(INFO) << _worker_number << ": Policy destructor end. Processed " << _next_calls << " calls that took " << _elapsed.count() << "s. Per call: " << _elapsed.count()/_next_calls << std::flush;
                }
            }

            Eigen::VectorXd pose_to_vector(const geometry_msgs::Pose& pose) const {
                Eigen::VectorXd response (7);
                Eigen::Vector3d position;
                tf::pointMsgToEigen(pose.position, position);
                response.head(3) = position;
                response(3) = pose.orientation.x;
                response(4) = pose.orientation.y;
                response(5) = pose.orientation.z;
                response(6) = pose.orientation.w;
                return response;
            }

            void mg_conf_from_ros_msgs(motion_generator_configuration::mg_conf& mg_conf, const SkirosNextActionResponse& res) {
                tf::pointMsgToEigen(res.ee_target_pose.position, mg_conf._pos);
                tf::quaternionMsgToEigen(res.ee_target_pose.orientation, mg_conf._rot);
                // Stiffness
                mg_conf._cart_stiffness[0] = res.cart_stiffness.force.x;
                mg_conf._cart_stiffness[1] = res.cart_stiffness.force.y;
                mg_conf._cart_stiffness[2] = res.cart_stiffness.force.z;
                mg_conf._cart_stiffness[3] = res.cart_stiffness.torque.x;
                mg_conf._cart_stiffness[4] = res.cart_stiffness.torque.y;
                mg_conf._cart_stiffness[5] = res.cart_stiffness.torque.z;
                mg_conf._nullspace_stiffness = res.nullspace_stiffness;
                // Wrench
                mg_conf._force[0] = res.ee_target_wrench.force.x;
                mg_conf._force[1] = res.ee_target_wrench.force.y;
                mg_conf._force[2] = res.ee_target_wrench.force.z;
                mg_conf._torque[0] = res.ee_target_wrench.torque.x;
                mg_conf._torque[1] = res.ee_target_wrench.torque.y;
                mg_conf._torque[2] = res.ee_target_wrench.torque.z;
                // Overlay motion
                mg_conf._overlay._overlay_motion = motion_generator_configuration::string_to_motion(res.overlay.motion);
                mg_conf._overlay._overlay_d[0] = res.overlay.dir.x;
                mg_conf._overlay._overlay_d[1] = res.overlay.dir.y;
                mg_conf._overlay._overlay_d[2] = res.overlay.dir.z;
                mg_conf._overlay._allow_decrease = res.overlay.allow_decrease;
                mg_conf._overlay._path_distance = res.overlay.path_distance;
                mg_conf._overlay._path_velocity = res.overlay.path_velocity;
                mg_conf._overlay._radius = res.overlay.radius;
                // Tool
                mg_conf._tool = res.tool;
            }

            void call_service(skireil::SkirosNextAction& srv_msg) {
                while (!_ac_next_action || !_ac_next_action.call(srv_msg)) {
                    LOG_EVERY_N(ERROR, 200) << _worker_number << ": Next action service call failed. Will retry forever.";
                    _ac_next_action.shutdown();
                    _ac_next_action = _node_handle.serviceClient<skireil::SkirosNextAction>(_worker_topic, true);
                    ros::Duration(0.05).sleep();
                }
            }

            void clear_msg_parameters() {
                _srv_msg.request.params_float.clear();
                _srv_msg.request.params_int.clear();
                _srv_msg.request.params_string.clear();
            }

            bool add_msg_parameters() {
                clear_msg_parameters();
                for (const auto& p : _params) {
                    switch(p->getDType()) {
                        case parameters::Int: {
                            skireil::ParamInt pe;
                            pe.name = p->getName();
                            pe.key = p->getKey();
                            pe.value = static_cast<int>(static_cast<parameters::Param<int>*>(p.get())->getVal());
                            _srv_msg.request.params_int.push_back(pe);
                            break;
                        }
                        case parameters::Float: {
                            skireil::ParamFloat pe;
                            pe.name = p->getName();
                            pe.key = p->getKey();
                            pe.value = static_cast<double>(static_cast<parameters::Param<float>*>(p.get())->getVal());
                            _srv_msg.request.params_float.push_back(pe);
                            break;
                        }
                        case parameters::String: {
                            skireil::ParamString pe;
                            pe.name = p->getName();
                            pe.key = p->getKey();
                            pe.value = static_cast<parameters::Param<std::string>*>(p.get())->getVal();
                            _srv_msg.request.params_string.push_back(pe);
                            break;
                        }
                        default:
                            parameters::fatalError("Unsupported parameter type for Eigen conversion.");
                    }
                }
                return true;
            }

            Eigen::VectorXd next(const Eigen::VectorXd& state, const double time = 0.0)
            {
                if (!_has_connection) {
                    LOG(FATAL) << "Asked to get next action, but there's no connection.";
                }
                // LOG(INFO) << _worker_number << ": Start next() function" << std::flush;
                auto start = std::chrono::high_resolution_clock::now();
                _srv_msg.request.type = _req_type;

                // Populate parameters
                if (_req_type == skireil::SkirosNextAction::Request::START_TASK) {
                    add_msg_parameters();
                    _srv_msg.request.skill_name = Params::bt_policy::application_name();
                } else {
                    clear_msg_parameters();
                }

                // Update robot ee position, joint configuration and time
                _simulated_robot->set_positions(state.head(7), Params::skireil::robot_dof());
                _srv_msg.request.robot_state.position = std::vector<double>(state.data(), state.data()+7);
                tf::poseEigenToMsg(_simulated_robot->skeleton()->getBodyNode(Params::skireil::robot_end_effector())->getTransform(), _srv_msg.request.ee_pose);
                _srv_msg.request.sim_time = time;

                call_service(_srv_msg);

                bt_response = _srv_msg.response.bt_response;

                // Process action
                motion_generator_configuration::mg_conf mg_conf;
                mg_conf_from_ros_msgs(mg_conf, _srv_msg.response);

                _req_type = skireil::SkirosNextAction::Request::TICK_TASK;
                auto finish = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = finish - start;
                _elapsed += elapsed;
                _next_calls++;
                // LOG(INFO) << _worker_number << ": End next() function." << std::flush;
                return mg_conf.to_eigen_vector();
            }

            void set_random_policy()
            {
                LOG(ERROR) << "SkiROSPolicy does not support random.";
            }

            bool random() const
            {
                return false;
            }

            void set_params(const ParamVec& params)
            {
                _params = params;
                // The first time we need to start the task. After that we tick only.
                // It is done here, because this is called before "next"
                _req_type = skireil::SkirosNextAction::Request::START_TASK;
            }

            ParamVec params() const
            {
                return _params;
            }

            void set_robot(std::shared_ptr<robot_dart::Robot> simulated_robot) {
                _simulated_robot = simulated_robot;
            }
            int bt_response{0};

        protected:
            ros::NodeHandle _node_handle{ros::NodeHandle("~")};
            ros::ServiceClient _ac_worker_manager;
            ros::ServiceClient _ac_next_action;
            skireil::SkirosNextAction _srv_msg;
            ParamVec _params;
            std::shared_ptr<robot_dart::Robot> _simulated_robot;
            int _worker_number{0};
            std::string _worker_topic;
            int _req_type{0};
            std::chrono::duration<double> _elapsed;
            unsigned int _next_calls{0};
            bool _has_connection{true};
        };
    } // namespace policy
} // namespace skireil
#endif
