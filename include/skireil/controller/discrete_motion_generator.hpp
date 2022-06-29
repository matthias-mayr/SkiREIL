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
#ifndef SKIREL_DISCRETE_MOTION_GENERATOR
#define SKIREL_DISCRETE_MOTION_GENERATOR

#include <cartesian_trajectory_generator/cartesian_trajectory_generator_base.h>
#include <cartesian_trajectory_generator/velocity_functions.h>
#include <cartesian_trajectory_generator/overlay_functions.h>

#include <memory>

#include <skireil/controller/motion_generator_configuration.hpp>

namespace motion_generator
{

template <typename Params>
class DiscreteMotionGenerator
{
public:
    DiscreteMotionGenerator()
    {
        double trans_v_max_{ 0.2 };
        double rot_v_max_{ 0.5 };
        double trans_a_{ 0.3 };
        double rot_a_{ 0.5 };
        double trans_d_{ 0.15 };
        double rot_d_{ 0.35 };
        bool synced{ true };

        // Set base parameters
        auto t = ctg_.get_translation_obj();
        t->set_acceleration(trans_a_);
        t->set_deceleration(trans_d_);
        t->set_v_max(trans_v_max_);
        auto r = ctg_.get_rotation_obj();
        r->set_acceleration(rot_a_);
        r->set_deceleration(rot_d_);
        r->set_v_max(rot_v_max_);
        ctg_.set_synchronized(synced);
    }

    void updateConfiguration(const motion_generator_configuration::mg_conf& conf, double t)
    {
        if (conf != last_conf_) {
            if (conf._overlay != last_conf_._overlay) {
                updateOverlay(conf, t);
            }
            if (!conf.pose_equal(last_conf_)) {
                this->requested_position_ = conf._pos;
                this->requested_orientation_ = conf._rot;
                updateGoal(t);
            }
            last_conf_ = conf;
        }
    }

    void updateConfiguration(const motion_generator_configuration::mg_conf& conf, double t, const Eigen::Vector3d& trans, const Eigen::Quaterniond& rot) {
        updatePose(trans, rot);
        updateConfiguration(conf, t);
    }

    void updatePose(const Eigen::Vector3d trans, const Eigen::Quaterniond& rot) {
        this->trans_ = trans;
        this->rot_ = rot;
    }

    bool updateOverlay(const motion_generator_configuration::mg_conf& conf, double t)
    {
        if (overlay_f_)
        {
            overlay_fade_ = overlay_f_->get_translation(t - overlay_start_);
        }
        if (motion_generator_configuration::motion_to_string(conf._overlay._overlay_motion) == "archimedes")
        {
            auto o = std::make_shared<cartesian_trajectory_generator::archimedes_spiral>();
            overlay_f_ = o;
            o->set_allow_decrease(conf._overlay._allow_decrease);
            o->set_direction(conf._overlay._overlay_d);
            o->set_max_radius(conf._overlay._radius);
            o->set_path_velocity(conf._overlay._path_velocity);
            o->set_path_distance(conf._overlay._path_distance);
            overlay_start_ = t;
        }
        else
        {
            overlay_f_.reset();
        }
        if (!first_goal_)
        {
            requested_position_ = trans_;
            requested_orientation_ = rot_;
            updateGoal(t);
        }
        return true;
    }

    void overlayFadeOut(Eigen::Vector3d &pos)
    {
        double norm = overlay_fade_.norm();
        if (norm > 0.0)
        {
            double diff = Params::skireil::dt() * 0.25 * trans_v_max_;
            if (diff > norm)
            {
                overlay_fade_ = Eigen::Vector3d::Zero();
            }
            else
            {
                overlay_fade_ = overlay_fade_ * (norm - diff) / norm;
            }
            pos += overlay_fade_;
        }
    }

    std::pair<Eigen::Vector3d, Eigen::Quaterniond> getPose(const Eigen::Vector3d& trans, const Eigen::Quaterniond& rot, double t)
    {
        this->updatePose(trans, rot);
        return this->getPose(t);
    }

    std::pair<Eigen::Vector3d, Eigen::Quaterniond> getPose(double t)
    {
        trajectory_t_ = t - traj_start_;
        overlay_t_ = t - overlay_start_;
        Eigen::Vector3d pos = ctg_.get_position(trajectory_t_);
        Eigen::Quaterniond rot = ctg_.get_orientation(trajectory_t_);
        if (overlay_f_)
        {
            pos += overlay_f_->get_translation(overlay_t_);
        }
        overlayFadeOut(pos);
        return std::make_pair(pos, rot);
    }

    void updateGoal(double t)
    {
        first_goal_ = true;
        ctg_.updateGoal(trans_, rot_, requested_position_, requested_orientation_);
        traj_start_ = t;
    }

private:
    cartesian_trajectory_generator::cartesian_trajectory_generator_base<cartesian_trajectory_generator::constant_acceleration,
                                        cartesian_trajectory_generator::constant_acceleration>
        ctg_;
    motion_generator_configuration::mg_conf last_conf_;
    Eigen::Vector3d requested_position_;
    Eigen::Quaterniond requested_orientation_;
    Eigen::Vector3d trans_;
    Eigen::Quaterniond rot_;
    std::shared_ptr<cartesian_trajectory_generator::overlay_base> overlay_f_;
    Eigen::Vector3d overlay_fade_{ Eigen::Vector3d::Zero() };
    bool first_goal_{ false };

    std::string frame_name_;
    std::string ee_link_;
    double trans_v_max_{ 0 };
    double trajectory_t_{ 0. };
    double overlay_t_{ 0. };
    double traj_start_{0.};
    double overlay_start_{0.};
};

}  // namespace motion_generator

#endif