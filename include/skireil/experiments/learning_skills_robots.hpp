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
#ifndef SKIREIL_LEARNING_SKILLS_ROBOTS_HPP
#define SKIREIL_LEARNING_SKILLS_ROBOTS_HPP

#include <memory>
#include <exception>

#include <robot_dart/robot.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/FreeJoint.hpp>

#include <skireil/utils/json.hpp>

namespace learning_skills {
    namespace robots {
        using json = nlohmann::json;

        std::shared_ptr<robot_dart::Robot> load_robot(const std::string& file, const std::vector<std::pair<std::string, std::string>>& packages, const std::string& name) {
            std::shared_ptr<robot_dart::Robot> robot;
            try {
                robot = std::make_shared<robot_dart::Robot>(file, packages, name);
            } catch (robot_dart::Assertion& a) {
                std::string reason = a.what();
                reason.append("\nThis also happens if the symlink is not re-established. Was 'scripts/build/configure.sh' run?");
                throw std::runtime_error(reason);
            }
            return robot;
        }

        template <typename Params>
        std::shared_ptr<robot_dart::Robot> bh_rss (const json& j) {
            // We need packages to desribe where the stl files are located.
            // They are copied into SkiREIL to avoid dependencies.
            std::vector<std::pair<std::string, std::string>> packages;
            packages.push_back(std::make_pair(std::string("bh_description"), std::string(RESPATH)));
            packages.push_back(std::make_pair(std::string("iiwa_description"), std::string(RESPATH)));
            std::string robot_file {std::string(RESPATH) + "/URDF/iiwa_with_peg/iiwa_left"};

            

            if (j["collisions"].get<std::string>() == "mesh") {
                robot_file += "_cad";
            }
            if (j["tool"].get<std::string>() == "peg_5mm") {
                robot_file += "_5mm";
                Params::skireil::set_robot_end_effector("peg");
            } else if (j["tool"].get<std::string>() == "peg_3mm") {
                robot_file += "_3mm";
                Params::skireil::set_robot_end_effector("peg");
            } else if (j["tool"].get<std::string>() == "peg_1mm") {
                robot_file += "_1mm";
                Params::skireil::set_robot_end_effector("peg");
            } else {
                Params::skireil::set_robot_end_effector("bh_link_ee");
            }
            robot_file += ".urdf";
            std::shared_ptr<robot_dart::Robot> robot = load_robot(robot_file, packages, "arm");
            Params::skireil::set_robot_dof(robot->dof_names(true, true, true));
            if (Params::skireil::robot_dof().size() != 7) {
                throw std::runtime_error("Expected 7 DOF, got " + std::to_string(Params::skireil::robot_dof().size()));
            }
            robot->set_actuator_types(Params::dart_policy_control::joint_type());

            Params::skireil::set_action_dim(19);
            Params::skireil::set_command_dim(7);
            Params::skireil::set_model_input_dim(14);
            Params::skireil::set_model_pred_dim(14);
            return robot;
        }

        template <typename Params>
        std::shared_ptr<robot_dart::Robot> bh_rss_2f_gripper (const json& j) {
            // We need packages to desribe where the stl files are located.
            // They are copied into skireil to avoid dependencies.
            std::vector<std::pair<std::string, std::string>> packages;
            packages.push_back(std::make_pair(std::string("bh_description"), std::string(RESPATH)));
            packages.push_back(std::make_pair(std::string("iiwa_description"), std::string(RESPATH)));
            std::string robotiq_f {std::string(RESPATH) + "/meshes/robotiq"};
            packages.push_back(std::make_pair(std::string("robotiq_2f_85_gripper_visualization"), robotiq_f));
            std::string robot_file {std::string(RESPATH) + "/URDF/iiwa_left"};

            Params::skireil::set_robot_end_effector("bh_link_ee");

            if (j["collisions"].get<std::string>() == "mesh") {
                robot_file += "_cad";
            }
            robot_file += ".urdf";
            std::shared_ptr<robot_dart::Robot> manipulator = load_robot(robot_file, packages, "arm");
            Params::skireil::set_robot_dof(manipulator->dof_names(true, true, true));
            if (Params::skireil::robot_dof().size() != 7) {
                throw std::runtime_error("Expected 7 DOF, got " + std::to_string(Params::skireil::robot_dof().size ()));
            }
            manipulator->set_actuator_types(Params::dart_policy_control::joint_type());

            std::string gripper_file {std::string(RESPATH) + "/URDF/robotiq_2f_85.urdf"};
            std::shared_ptr<robot_dart::Robot> gripper_skel = load_robot(gripper_file, packages, "gripper");
            // Params::skireil::set_tool_dof(std::vector<std::string>{"finger_joint"});

            /* Attach gripper to manipulator */
            dart::dynamics::WeldJoint::Properties properties = dart::dynamics::WeldJoint::Properties();
            properties.mT_ChildBodyToJoint.translation() = Eigen::Vector3d(0, 0, 0.0001);
            gripper_skel->skeleton()->getRootBodyNode()->moveTo<dart::dynamics::WeldJoint>(manipulator->skeleton()->getBodyNode("bh_link_ee"), properties);
            manipulator->update_joint_dof_maps();
            /* The gripper is controlled in velocity mode: servo actuator */
            manipulator->set_actuator_type("passive", "finger_joint");

            Params::skireil::set_action_dim(19);
            Params::skireil::set_command_dim(7);
            Params::skireil::set_model_input_dim(14);
            Params::skireil::set_model_pred_dim(14);
            return manipulator;
        }

        template <typename Params>
        std::shared_ptr<robot_dart::Robot> bh_rss_polyhedron (const json& j) {
            // We need packages to desribe where the stl files are located.
            // They are copied into skireil to avoid dependencies.
            std::vector<std::pair<std::string, std::string>> packages;
            packages.push_back(std::make_pair(std::string("bh_description"), std::string(RESPATH)));
            packages.push_back(std::make_pair(std::string("iiwa_description"), std::string(RESPATH)));
            std::string compatible_path = RESPATH;
            compatible_path.append("/..");
            packages.push_back(std::make_pair(std::string("skireil"), compatible_path));
            std::string robot_file {std::string(RESPATH) + "/URDF/iiwa_with_peg/iiwa_left"};

            Params::skireil::set_robot_end_effector("bh_link_ee");

            if (j["collisions"].get<std::string>() == "mesh") {
                robot_file += "_cad";
            }
            if (j["tool"].get<std::string>() == "peg_10mm_hr") {
                robot_file += "_10mm_hr";
                Params::skireil::set_robot_end_effector("peg");
            } else if (j["tool"].get<std::string>() == "peg_20mm_hr") {
                robot_file += "_20mm_hr";
                Params::skireil::set_robot_end_effector("peg");
            } else if (j["tool"].get<std::string>() == "peg_30mm_hr") {
                robot_file += "_30mm_hr";
                Params::skireil::set_robot_end_effector("peg");
            } else if (j["tool"].get<std::string>() == "peg_45mm_hr") {
                robot_file += "_45mm_hr";
                Params::skireil::set_robot_end_effector("peg");
            } else if (j["tool"].get<std::string>() == "peg_70mm_hr") {
                robot_file += "_70mm_hr";
                Params::skireil::set_robot_end_effector("peg");
            } else {
                Params::skireil::set_robot_end_effector("bh_link_ee");
            }
            robot_file += ".urdf";
            std::shared_ptr<robot_dart::Robot> manipulator = load_robot(robot_file, packages, "arm");
            Params::skireil::set_robot_dof(manipulator->dof_names(true, true, true));
            if (Params::skireil::robot_dof().size() != 7) {
                throw std::runtime_error("Expected 7 DOF, got " + std::to_string(Params::skireil::robot_dof().size ()));
            }
            manipulator->set_actuator_types(Params::dart_policy_control::joint_type());

            std::string polyhedron_file {std::string(RESPATH) + "/URDF/polyhedron_30cm.urdf"};
            std::shared_ptr<robot_dart::Robot> polyhedron_skel = load_robot(polyhedron_file, packages, "polyhedron");

            /* Attach polyhedron to manipulator */
            dart::dynamics::FreeJoint::Properties properties = dart::dynamics::FreeJoint::Properties();
            // Spawn polyhedron 5mm above the table
            Eigen::Vector3d ph_pos = Eigen::Vector3d(-0.2, 0.35, 0.675);
            if (Params::skireil::domain_randomization()) {
                // Randomly add offset in x and y direction
                double sigma{0.007};
                for (size_t i = 0; i <= 1; i++) {
                    double offset = skireil::utils::gaussian_rand(0, sigma);
                    offset = std::max(-sigma, std::min(offset, sigma));
                    ph_pos[i] = ph_pos[i] + offset;
                }
            }
            
            properties.mT_ParentBodyToJoint.translation() = ph_pos;
            Eigen::Matrix3d m;
            double angle1{3.14};
            double angle2{0.0};
            double angle3{0.4625122};
            if (Params::skireil::domain_randomization()) {
                double sigma{0.01};
                double offset = skireil::utils::gaussian_rand(0, sigma);
                offset = std::max(-sigma, std::min(offset, sigma));
                angle1 += offset;
                offset = skireil::utils::gaussian_rand(0, sigma);
                offset = std::max(-sigma, std::min(offset, sigma));
                angle2 += offset;
                offset = skireil::utils::gaussian_rand(0, sigma);
                offset = std::max(-sigma, std::min(offset, sigma));
                angle3 += offset;
            }
            m = Eigen::AngleAxisd(angle1, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(angle2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(angle3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            properties.mT_ParentBodyToJoint.linear() = m;
            properties.mName = "polyhedron_joint";
            polyhedron_skel->skeleton()->getRootBodyNode()->moveTo<dart::dynamics::FreeJoint>(manipulator->skeleton()->getBodyNode("base_link"), properties);
            manipulator->update_joint_dof_maps();
            const double plastic_friction{0.4};
            manipulator->set_friction_coeff("polyhedron_link", plastic_friction);
            manipulator->set_friction_coeff("peg_base", plastic_friction);
            manipulator->set_actuator_type("passive", "polyhedron_joint");
            Params::skireil::set_state_objects(std::vector<std::string>{"polyhedron_link"});

            Params::skireil::set_action_dim(19);
            Params::skireil::set_command_dim(7);
            Params::skireil::set_model_input_dim(21);
            Params::skireil::set_model_pred_dim(21);
            return manipulator;
        }

    } // namespace robots
} // namespace learning_skills

#endif