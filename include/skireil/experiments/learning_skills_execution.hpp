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
#ifndef SKIREIL_LEARNING_SKILLS_EXECUTION
#define SKIREIL_LEARNING_SKILLS_EXECUTION

#include <stdlib.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <chrono>
#include <exception>
#include <memory>

#include <robot_dart/robot.hpp>

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>

#include <skireil/utils/utils.hpp>


using namespace std::chrono;

#define degreesToRadians(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define radiansToDegrees(angleRadians) ((angleRadians) * 180.0 / M_PI)

namespace skireil {
    namespace execution {
        bool reset_to_start_pos() {
            // Reset to start position
            int positions = 5;
            Eigen::VectorXd r_nr_v = skireil::utils::uniform_rand(1);
            double r_nr = r_nr_v[0] * positions;
            r_nr += 0.5;
            int index = static_cast<int>(std::round(r_nr));
            // Cover edge cases:
            if (index < 1) {
                index = 1;
            } else if (index > positions) {
                index = static_cast<int>(positions);
            }
            // Python file takes the start position as an argument
            std::string cmd("python ");
            cmd += "./scripts/experiments/reset_iiwa.py";
            cmd += " peg" + std::to_string(index);

            LOG(INFO) << "Executing command: " << cmd << std::endl;
            // Call python script and wait for it to finish
            struct skireil::utils::popen2 start_script;
            skireil::utils::popen2(cmd.c_str(), &start_script);
            waitpid(start_script.child_pid, NULL, 0 );
            return true;
        }
    }
}

template <class PolicyController, typename Params>
struct learning_skills_execution {

    // Translates a JointState message to our state descriptions
    void states_callback(const sensor_msgs::JointStatePtr& states) {
        _real_states_stamp = states->header.stamp;
        _real_states[0] = states->position[0];
        _real_states[1] = states->position[1];
        _real_states[2] = states->position[2];
        _real_states[3] = states->position[3];
        _real_states[4] = states->position[4];
        _real_states[5] = states->position[5];
        _real_states[6] = states->position[6];
        _real_states[7] = states->velocity[0];
        _real_states[8] = states->velocity[1];
        _real_states[9] = states->velocity[2];
        _real_states[10] = states->velocity[3];
        _real_states[11] = states->velocity[4];
        _real_states[12] = states->velocity[5];
        _real_states[13] = states->velocity[6];
    }

    // Saves if the robot can be commanded
    void status_callback(const std_msgs::BoolConstPtr& status) {
        _commanding_status = status->data;
    }

    template <typename T>
    std::string vector_to_string(const std::vector<T>& vector) {
        std::stringstream param_str;
        for (const auto& el : vector) {
            param_str << el <<  ", ";
        }
        return param_str.str();
    }

    learning_skills_execution() {
        _pub_reference_pose = _node_handle.advertise<geometry_msgs::PoseStamped>("/bh/CartesianImpedance_trajectory_controller/reference_pose", 2);
        _pub_stiffness = _node_handle.advertise<geometry_msgs::WrenchStamped>("/bh/CartesianImpedance_trajectory_controller/set_cartesian_stiffness", 2);
        _pub_wrench = _node_handle.advertise<geometry_msgs::WrenchStamped>("/bh/CartesianImpedance_trajectory_controller/set_cartesian_wrench", 2);
        _sub_states = _node_handle.subscribe("/bh/joint_states", 1, &learning_skills_execution::states_callback, this);
        _sub_status = _node_handle.subscribe("/bh/commanding_status", 1, &learning_skills_execution::status_callback, this);
    }

    void set_robot_state(const std::shared_ptr<robot_dart::Robot>& robot, const Eigen::VectorXd& state) const
    {
        // Todo: These "7" should not be here. One can use robot_dof.size().
        robot->set_positions(state.head(7), Params::skireil::robot_dof());
        robot->set_velocities(state.segment(7, 7), Params::skireil::robot_dof());
    }

    void save_data(std::shared_ptr<PolicyController> policy_control, const std::string& traj_sparse_filename, const std::string& traj_dense_filename, const std::string& ee_dense_filename) {
        // Saving the data
        skireil::utils::save_traj_to_file(traj_sparse_filename, policy_control->get_states(), policy_control->get_commands());
        std::ofstream outfile;
        outfile.open(traj_sparse_filename, std::ios_base::app); // append instead of overwrite
        outfile << "finish_time:" << policy_control->finish_time;
        LOG(INFO) << "finish_time:" << policy_control->finish_time;
        skireil::utils::save_traj_to_file(traj_dense_filename, policy_control->get_dense_states(), policy_control->get_dense_commands());
        skireil::utils::save_traj_to_file(ee_dense_filename, policy_control->get_dense_ee_pos(), policy_control->get_dense_ee_rot());
    }

    std::shared_ptr<PolicyController> configure_controller(std::shared_ptr<robot_dart::Robot> robot, const ParamVec& params) {
        std::shared_ptr<PolicyController> policy_control = std::make_shared<PolicyController>();
        policy_control->set_robot_in_policy(robot->clone());
        policy_control->configure(params);
        robot->add_controller(policy_control);
        return policy_control;
    }

    void publish_reference_pose(const Eigen::VectorXd& v) {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        msg.pose.position.x = v(0);
        msg.pose.position.y = v(1);
        msg.pose.position.z = v(2);
        msg.pose.orientation.x = v(3);
        msg.pose.orientation.y = v(4);
        msg.pose.orientation.z = v(5);
        msg.pose.orientation.w = v(6);
        _pub_reference_pose.publish(msg);
    }

    void publish_cartesian_stiffness(const Eigen::VectorXd& v) {
        geometry_msgs::WrenchStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        msg.wrench.force.x = v(0);
        msg.wrench.force.y = v(1);
        msg.wrench.force.z = v(2);
        msg.wrench.torque.x = v(3);
        msg.wrench.torque.y = v(4);
        msg.wrench.torque.z = v(5);
        _pub_stiffness.publish(msg);
    }

    void publish_cartesian_wrench(Eigen::VectorXd v) {
        // for (size_t i = 0; i < 3; i++)
        // {
        //     if (v(i) > 0.0) {
        //         v(i) += 2.0;
        //     } else if (v(i) < 0.0) {
        //         v(i) -= 2.0;
        //     }
        // }
        if (v(2) != 0.0) {
            v(2) = v(2) - 5.0;
        }
        geometry_msgs::WrenchStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "world";
        msg.wrench.force.x = v(0);
        msg.wrench.force.y = v(1);
        msg.wrench.force.z = v(2);
        msg.wrench.torque.x = v(3);
        msg.wrench.torque.y = v(4);
        msg.wrench.torque.z = v(5);
        _pub_wrench.publish(msg);
    }

    bool run(std::weak_ptr<PolicyController> policy_control_pointer, std::weak_ptr<robot_dart::Robot> robot_pointer, bool exit_on_success = false) {
        if (policy_control_pointer.expired()) {
            LOG(ERROR) << "Policy control pointer has expired. Aborting execution.";
            return false;
        }
        if (robot_pointer.expired()) {
            LOG(ERROR) << "Robot pointer has expired. Aborting execution.";
            return false;
        }
        auto policy_control = policy_control_pointer.lock();
        auto robot = robot_pointer.lock();
        double t{0};
        Eigen::VectorXd commands, action;
        // Check if we can connect to the robot with timeout
        auto start = ros::Time::now();
        while (_real_states[0] == 0.0 || (ros::Time::now() - _real_states_stamp) > ros::Duration(0.1)) {
            ros::spinOnce();
            if ((ros::Time::now()-start).toSec() > 5.) {
                LOG(ERROR) << "Could not connect to the robot. Quitting.";
                return false;
            }
        }

        ros::Rate r(1.0/Params::skireil::dt());
        // The first time takes longer, so we do it outside the control loop
        set_robot_state(robot, _real_states);
        commands = policy_control->calculate(t);
        start = ros::Time::now();

        // Execute the policy on the real system until time T
        while (t < Params::skireil::T()) {
            r.sleep();
            if (r.cycleTime().toSec() > 1.5*r.expectedCycleTime().toSec()) {
                LOG(WARNING) << "Loop took longer: " << r.cycleTime() << " ms";
            }
            t = (ros::Time::now()-start).toSec();
            ros::spinOnce();
            if (!ros::ok() || !_commanding_status) {
                LOG(ERROR) << "Error when executing policy. Exiting.";
                return false;
            }
            if (exit_on_success && policy_control->finish_time > 0) {
                LOG(INFO) << "Successfully finished task. Exiting.";
                return true;
            }
            if (Params::meta_conf::verbose()) {
                LOG_EVERY_N(INFO, 1.0/Params::skireil::dt()) << "t: " << t << "\n";
            }

            set_robot_state(robot, _real_states);
            commands = policy_control->calculate(t);
            //LOG(INFO) << "Real states: " << _real_states.array().format(OneLine);
            // auto start = high_resolution_clock::now();
            
            // Get action and send it out
            action = policy_control->get_last_command();
            publish_reference_pose(action.head(7));
            publish_cartesian_wrench(action.segment(7,6));
            publish_cartesian_stiffness(action.segment(13,6));

            // auto stop = high_resolution_clock::now();
            // auto duration = duration_cast<milliseconds>(stop - start);
            // LOG(INFO) << "Calculate took in ms: " << duration.count();
        }
        return true;
    }

    ros::NodeHandle _node_handle{ros::NodeHandle("~")};
    ros::Publisher _pub_reference_pose;
    ros::Publisher _pub_stiffness;
    ros::Publisher _pub_wrench;
    ros::Subscriber _sub_states;
    ros::Subscriber _sub_status;
    Eigen::VectorXd _real_states{Eigen::VectorXd::Zero(14)};
    ros::Time _real_states_stamp;
    bool _commanding_status{false};
    const std::string _experiment;
};

#endif