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
#ifndef SKIREIL_LEARNING_SKILLS_POLICY_CONTROL_HPP
#define SKIREIL_LEARNING_SKILLS_POLICY_CONTROL_HPP

#include <skireil/system/dart_system.hpp>
#include <skireil/parameters.hpp>
#include <cartesian_impedance_controller/cartesian_impedance_controller.h>
#include <skireil/controller/discrete_motion_generator.hpp>
#include <skireil/controller/motion_generator_configuration.hpp>


using ParamVec = std::vector<std::shared_ptr<skireil::parameters::ParamBase>>;
// Controls the robot in DART and handles the queries to the policy.
template <typename Params, typename Policy>
struct PolicyControl : public skireil::system::BaseDARTPolicyControl<Params, Policy> {
    using base_t = skireil::system::BaseDARTPolicyControl<Params, Policy>;

    PolicyControl() : base_t() {
        // We allow torque changes of 1000Nm/s
        _cart_controller.setMaxTorqueDelta(1000.0 * Params::skireil::control_dt());
        _cart_controller.setFiltering(1.0/Params::skireil::control_dt(), 1.0, 1.0, 1., 1.0);
    }
    PolicyControl(const Eigen::VectorXd& ctrl) : base_t(ctrl) {
        PolicyControl();
    }

    void configure(const ParamVec& pv)
    {
        this->_prev_time = 0.0;
        this->_t = 0.0;
        this->_first = true;

        this->_policy.set_params(pv);

        this->_states.clear();
        this->_noiseless_states.clear();
        this->_coms.clear();
        this->_dense_states.clear();
        this->_dense_coms.clear();
        this->_applied_residual_mean.clear();
        this->_applied_residual_variance.clear();
        if (Params::skireil::tool_dof().empty()) {
            this->_has_tool = false;
        }
        this->_prev_commands = Eigen::VectorXd(Params::skireil::command_dim());
        if (Params::skireil::command_dim() == (Params::skireil::robot_dof().size() + Params::skireil::tool_dof().size())) {
            this->_active = true;
        } else {
            LOG(WARNING) << "Command size is not 7 or 8. Did not activate controller.";
        }
    }

    void set_fixed_action(const Eigen::VectorXd& action) {
        this->_fixed_action = action;
    }

    void process_command(const Eigen::VectorXd& command, double t, bool hard_set_desired = false) {
        motion_generator_configuration::mg_conf conf = motion_generator_configuration::mg_conf(command);
        if (_last_conf != conf || hard_set_desired) {
            _stiffness_target = conf.get_stiffnesses();
            _wrench_target = conf.get_wrench();
            _mg.updateConfiguration(conf, t);
            if (hard_set_desired) {
                _stiffness = _stiffness_target;
            }
            set_tool_target(conf._tool);
        }
        _last_conf = conf;
    }

    void update_wrench() {
        _wrench.head(3) = sat_change_v<Eigen::VectorBlock<Eigen::Matrix<double, 6, 1>, -1>, Eigen::Matrix<double, 3, 1>>(_wrench.head(3), _wrench_target.head(3), _force_change);
        _wrench.tail(3) = sat_change_v<Eigen::VectorBlock<Eigen::Matrix<double, 6, 1>, -1>, Eigen::Matrix<double, 3, 1>>(_wrench.tail(3), _wrench_target.tail(3), _torque_change);
    }

    void update_stiffnesses() {
        _stiffness.head(6) = sat_change_v<Eigen::VectorBlock<Eigen::Matrix<double, 7, 1>, -1>, Eigen::Matrix<double, 6, 1>>(_stiffness.head(6), _stiffness_target.head(6), _cart_stiffness_change);
        _stiffness[6] = sat_change(_stiffness_target[6], _stiffness[6], _ns_stiffness_change);
    }

    void set_tool_target(double dv) {
        int i = std::round(dv);
        if (i == -1 || i == 0) {
            // Gripper closed
            _gripper_desired_position = 0.3;
        } else if (i == 1) {
            // Gripper open
            _gripper_desired_position = 0.0;
        } else {
            LOG(WARNING) << "Got invalid tool control target: " << i;
        }
    }

    double gripper_control_command() {
        std::shared_ptr<robot_dart::Robot> robot = this->_robot.lock();
        return _p_gripper_gain * (_gripper_desired_position - CHECK_NOTNULL(robot)->positions(Params::skireil::tool_dof())[0]);
    }

    Eigen::VectorXd get_command(const Eigen::Matrix<double, 7, 1>& q, const Eigen::VectorXd& dq, const Eigen::VectorXd& currentWorldPosition, const Eigen::Quaterniond& currentWorldOrientationQ, const Eigen::MatrixXd& J, std::shared_ptr<robot_dart::Robot> robot) {
        if (this->_has_tool) {
            Eigen::VectorXd t(Params::skireil::command_dim());
            t << _cart_controller.calculateCommandedTorques(q, dq, currentWorldPosition, currentWorldOrientationQ, J), gripper_control_command();
            Eigen::VectorXd comp(Params::skireil::command_dim());
            comp << robot->coriolis_gravity_forces(Params::skireil::robot_dof()), Eigen::VectorXd::Zero(Params::skireil::tool_dof().size());
            return t + comp;
        } else {
            Eigen::VectorXd t(Params::skireil::command_dim());
            t << _cart_controller.calculateCommandedTorques(q, dq, currentWorldPosition, currentWorldOrientationQ, J);
            t += robot->coriolis_gravity_forces(Params::skireil::robot_dof());
            return t;
        }
    }

    Eigen::VectorXd calculate(double t)
    {
        this->_t = t;
        this->_update_func(t);
        const double dt = Params::skireil::dt();

        std::shared_ptr<robot_dart::Robot> robot = this->_robot.lock();
        const Eigen::Matrix<double, 7, 1> q = CHECK_NOTNULL(robot)->positions(Params::skireil::robot_dof());
        const Eigen::VectorXd dq = robot->velocities(Params::skireil::robot_dof());

        /* Get full Jacobian of our end-effector */
        const Eigen::MatrixXd J_dart = robot->jacobian(Params::skireil::robot_end_effector(), Params::skireil::robot_dof());
        const Eigen::MatrixXd J = this->_jacobian_perm * J_dart;

        /* Get current _state of the end-effector */
        const Eigen::MatrixXd currentWorldTransformation = robot->skeleton()->getBodyNode(Params::skireil::robot_end_effector())->getWorldTransform().matrix();
        const Eigen::VectorXd currentWorldPosition = currentWorldTransformation.block(0, 3, 3, 1);
        const Eigen::Quaterniond currentWorldOrientationQ (Eigen::Matrix3d(currentWorldTransformation.block(0, 0, 3, 3)));
        Eigen::VectorXd q_problem = get_state(this->_robot.lock());

        // Queries the policy every dt for a new command
        if ((this->_first || (this->_t - this->_prev_time - dt) > -Params::dart_system::sim_step() / 2.0)) {
            this->_noiseless_states.push_back(q_problem);
            q_problem = this->_add_noise(q_problem);
            _mg.updatePose(currentWorldPosition, currentWorldOrientationQ);
            std::pair<Eigen::Vector3d, Eigen::Quaterniond> ref_pose;
            if (!skip_next) {
            Eigen::VectorXd action = this->_policy.next(this->_policy_state(this->_tranform_state(q_problem)), t);
            if (this->_policy.bt_response == _bt_success && finish_time < 0) {
                finish_time = this->_t;
            }
            process_command(action, t, this->_first);
            update_wrench();
                update_stiffnesses();
                ref_pose = _mg.getPose(t);
            } else {
                this->set_from_problem_action(_fixed_action, &ref_pose, &this->_wrench, &this->_stiffness);
            }
            _cart_controller.setReferencePose(ref_pose.first, ref_pose.second);
            _cart_controller.applyWrench(_wrench);
            _cart_controller.setStiffness(_stiffness, true);
            this->_prev_commands = get_command(q, dq, currentWorldPosition, currentWorldOrientationQ, J, robot);
            this->_prev_control_time = this->_t;
            this->_states.push_back(q_problem);
            this->_coms.push_back(get_problem_action(t));
            this->_prev_time = this->_t;
            this->_first = false;
        } else if ((this->_t - this->_prev_control_time) > Params::skireil::control_dt()) {
            this->_prev_commands = get_command(q, dq, currentWorldPosition, currentWorldOrientationQ, J, robot);
            this->_prev_control_time = this->_t;
        }
        dense_logging(currentWorldPosition, currentWorldOrientationQ, q_problem);
        return this->_prev_commands;
    }

    template <typename T>
    T sat_change(const T& v, const T& v_target, double change) const {
        bool sign = v_target > v;
        double diff = std::min(std::abs(v - v_target), change);
        if (sign) {
            return v + diff;
        } else {
           return v - diff;
        }
    }

    template <typename V, typename V_new>
    V_new sat_change_v(const V& v, const V& v_target, double change) const {
        V_new v_new;
        for (int i = 0; i < v.size(); i++) {
            v_new[i] = sat_change(v[i], v_target[i], change);
        }
        return v_new;
    }

    void set_robot_in_policy(std::shared_ptr<robot_dart::Robot> simulated_robot) {
        this->_policy.set_robot(simulated_robot);
    }

    void set_goal(const Eigen::Vector3d goal) {
        _goal = goal;
    }

    void set_controller_step(const double step) {
        // cart_controller.set_step(step);
    }
    void set_controller_real_params() {
        // Does not do anything here.
    }
    
    void dense_logging(const Eigen::VectorXd& position, const Eigen::Quaterniond& orientation, const Eigen::VectorXd& q_problem) {
        _dense_states.push_back(q_problem);
        _dense_coms.push_back(this->_prev_commands);
        _dense_ee_pos.push_back(position);
        _dense_ee_rot.push_back(orientation.toRotationMatrix().eulerAngles(0, 1, 2));
    }

    void append_applied_residual(const Eigen::VectorXd& mean, const Eigen::VectorXd& variance) {
        _applied_residual_mean.push_back(mean);
        _applied_residual_variance.push_back(variance);
    }

    Eigen::VectorXd get_problem_action(double t) {
        Eigen::VectorXd action(Params::skireil::action_dim());
        std::pair<Eigen::Vector3d, Eigen::Quaterniond> pose = _mg.getPose(t);
        action << pose.first, pose.second.coeffs(), this->_wrench, this->_stiffness.head(6);
        if (static_cast<size_t>(action.size()) != Params::skireil::action_dim()) {
            throw std::runtime_error("Wrong action dimensionality. Expected " + std::to_string(Params::skireil::action_dim()) + " but got " + std::to_string(action.size()));
        }
        return action;
    }

    void set_from_problem_action(const Eigen::VectorXd& action, std::pair<Eigen::Vector3d, Eigen::Quaterniond>* ref_pose, Eigen::Matrix<double, 6, 1>* wrench, Eigen::Matrix<double, 7, 1>* stiffness) {
        if (static_cast<size_t>(action.size()) != Params::skireil::action_dim()) {
            throw std::runtime_error("Wrong action dimensionality. Expected " + std::to_string(Params::skireil::action_dim()) + " but got " + std::to_string(action.size()));
        }
        Eigen::Vector3d pos = action.head(3);
        Eigen::Quaterniond rot = Eigen::Quaterniond(action.segment(3, 4).data());
        *ref_pose = std::make_pair(pos, rot);
        *wrench = action.segment(7, 6);
        // The problem action only uses Cartesian stiffness
        *stiffness = Eigen::Matrix<double, 7, 1>::Zero();
        stiffness->head(6) = action.segment(13, 6);
    }

    Eigen::VectorXd get_object_poses(const std::shared_ptr<robot_dart::Robot>& robot) const
    {
        Eigen::VectorXd add_states(7 * Params::skireil::state_objects().size());
        for (size_t i = 0; i < Params::skireil::state_objects().size(); i++) {
            const auto bodyNode = robot->skeleton()->getBodyNode(Params::skireil::state_objects()[i]);
            if (bodyNode == NULL) {
                throw std::runtime_error("Requested pose of link that does not exist: " + Params::skireil::state_objects()[i]);
            }
            Eigen::MatrixXd currentWorldTransformationPh = bodyNode->getWorldTransform().matrix();
            Eigen::VectorXd currentWorldPositionPh = currentWorldTransformationPh.block(0, 3, 3, 1);
            Eigen::Quaterniond currentWorldOrientationQPh(Eigen::Matrix3d(currentWorldTransformationPh.block(0, 0, 3, 3)));
            add_states.segment(7 * i, 3) = currentWorldPositionPh;
            add_states.segment(7 * i + 3, 4) = currentWorldOrientationQPh.coeffs();
        }
        return add_states;
    }

    Eigen::VectorXd get_state(const std::shared_ptr<robot_dart::Robot>& robot) const
    {
        // TODO: Fix gripper integration
        Eigen::VectorXd state(Params::skireil::model_pred_dim());
        std::vector<std::string> dofs = Params::skireil::robot_dof();
        std::vector<std::string> tool_dof = Params::skireil::tool_dof();
        dofs.insert(dofs.end(), tool_dof.begin(), tool_dof.end());
        state << robot->positions(dofs), robot->velocities(dofs), get_object_poses(robot);
        return state;
    }

    std::vector<Eigen::VectorXd> get_dense_states() const
    {
        return _dense_states;
    }

    std::vector<Eigen::VectorXd> get_dense_commands() const
    {
        return _dense_coms;
    }

    std::vector<Eigen::VectorXd> get_dense_ee_pos() const
    {
        return _dense_ee_pos;
    }

    std::vector<Eigen::VectorXd> get_dense_ee_rot() const
    {
        return _dense_ee_rot;
    }

    std::vector<Eigen::VectorXd> get_applied_residual_mean() const
    {
        return _applied_residual_mean;
    }

    std::vector<Eigen::VectorXd> get_applied_residual_variance() const
    {
        return _applied_residual_variance;
    }

    std::shared_ptr<robot_dart::control::RobotControl> clone() const override
    {
        return std::make_shared<PolicyControl>(*this);
    }

    void set_skip_next() {
        skip_next = true;
    }

    double finish_time{-1};
    private:
        std::vector<Eigen::VectorXd> _dense_coms;
        std::vector<Eigen::VectorXd> _dense_states;
        std::vector<Eigen::VectorXd> _dense_ee_pos;
        std::vector<Eigen::VectorXd> _dense_ee_rot;
        std::vector<Eigen::VectorXd> _applied_residual_mean;
        std::vector<Eigen::VectorXd> _applied_residual_variance;
        Eigen::Vector3d _goal = Eigen::Vector3d::Zero();
        Eigen::VectorXd _fixed_action = Eigen::VectorXd::Zero(0);
        double _prev_control_time{0.0};
        bool skip_next = false;
        cartesian_impedance_controller::CartesianImpedanceController _cart_controller;
        bool _has_tool{true};
        double _gripper_desired_position{0.0};
        const double _p_gripper_gain = 100.0;
        motion_generator::DiscreteMotionGenerator<Params> _mg;
        const Eigen::VectorXi _perm_indices = (Eigen::Matrix<int, 6,1>() << 3,4,5,0,1,2).finished();
        const Eigen::PermutationMatrix<Eigen::Dynamic,6> _jacobian_perm{Eigen::PermutationMatrix<Eigen::Dynamic,6>(_perm_indices)};
        motion_generator_configuration::mg_conf _last_conf;
        const double _torque_change{5.0 * Params::skireil::dt()}; // Nm/s
        const double _force_change{10.0 * Params::skireil::dt()};  // N/s
        Eigen::Matrix<double, 6, 1> _wrench_target {Eigen::Matrix<double, 6, 1>::Zero()};
        Eigen::Matrix<double, 6, 1> _wrench {Eigen::Matrix<double, 6, 1>::Zero()};
        const double _cart_stiffness_change{500.0 * Params::skireil::dt()};
        const double _ns_stiffness_change{500.0 * Params::skireil::dt()};
        Eigen::Matrix<double, 7, 1> _stiffness_target {Eigen::Matrix<double, 7, 1>::Zero()};
        Eigen::Matrix<double, 7, 1> _stiffness {Eigen::Matrix<double, 7, 1>::Zero()};
        // SkiROS defines success as 1
        // BehaviorTrees-Cpp defines success as 2
        const int _bt_success {1};
};

#endif
