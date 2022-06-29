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
#ifndef SKIREIL_LEARNING_SKILLS_SCENES_HPP
#define SKIREIL_LEARNING_SKILLS_SCENES_HPP

#include <Eigen/Core>

#include <robot_dart/robot.hpp>
#include <robot_dart/robot_dart_simu.hpp>

#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>

namespace learning_skills {
    namespace scenes {
        template <typename RolloutInfo, typename Params>
        void scene_obstacle_and_peg(robot_dart::RobotDARTSimu& simu, const RolloutInfo& info)
        {
            // Add collision detector that works with meshes
            simu.world()->getConstraintSolver()->setCollisionDetector( dart::collision::FCLCollisionDetector::create());

            simu.world()->setGravity(Eigen::Vector3d(0, 0, -9.81));

            // Add goal marker
            Eigen::Vector6d goal_pose = Eigen::Vector6d::Zero();
            goal_pose.tail(3) = info.goal;
            // dims, pose, type, mass, color, name
            auto ellipsoid = robot_dart::Robot::create_ellipsoid({0.03, 0.03, 0.03}, goal_pose, "fixed", 1., dart::Color::Green(1.0), "goal_marker");
            // remove collisions from goal marker
            ellipsoid->skeleton()->getRootBodyNode()->setCollidable(false);
            // add ellipsoid to simu
            simu.add_robot(ellipsoid);

            // Insert table
            Eigen::Vector6d table_pose;
            table_pose << 0., 0., 0., 0.8, 0.4, 0.415;
            auto table = robot_dart::Robot::create_box({table_pose[3]*2, table_pose[4]*2, table_pose[5]*2}, table_pose, "fixed", 100., dart::Color::White(1.0), "table");
            simu.add_robot(table);

            // Insert workbox
            const double plastic_friction{0.45};            Eigen::Vector6d workbox_pose;
            workbox_pose << 0., 0., 0., -(0.942/2.0), 0.48, 0.3;
            auto workbox = robot_dart::Robot::create_box({std::abs(workbox_pose[3]*2), 2.0, std::abs(workbox_pose[5]*2)}, workbox_pose, "fixed", 100., dart::Color::Gray(1.0), "workbox");
            workbox->skeleton()->getRootBodyNode()->setCollidable(false);
            workbox->set_friction_coeff("workbox", plastic_friction);
            simu.add_robot(workbox);

            // Insert block with hole
            // Friction values for the box. Manually tuned.
            std::vector<std::pair<std::string, std::string>> packages;
            packages.push_back(std::make_pair(std::string("skireil"), std::string(RESPATH)));
            auto box_with_hole = std::make_shared<robot_dart::Robot>(std::string(RESPATH) + "/URDF/box_with_hole.urdf", packages, "box_with_hole");
            box_with_hole->set_friction_coeff("box_link", plastic_friction);
            // Set box position
            Eigen::Vector3d box_position = Eigen::Vector3d(-0.675, -0.075, 0.603);
            if (Params::skireil::domain_randomization()) {
                // Randomly add offset in x and y direction
                double sigma{0.007};
                for (size_t i = 0; i <= 1; i++) {
                    double offset = skireil::utils::gaussian_rand(0, sigma);
                    offset = std::max(-sigma, std::min(offset, sigma));
                    box_position[i] = box_position[i] + offset;
                }
            }
            Eigen::Isometry3d T;
            T.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d::Zero());
            T.translation() = box_position;
            auto joint = box_with_hole->skeleton()->getJoint("box_joint");
            joint->setTransformFromParentBodyNode(T);
            simu.add_robot(box_with_hole);

            // Insert obstacle
            Eigen::Vector6d engine_pose;
            engine_pose << 0., 0., 0., -0.52, 0.30, workbox_pose[5]*2+0.151;
            if (Params::skireil::domain_randomization()) {
                // Randomly add offset in x and y direction
                double sigma{0.02};
                for (size_t i = 3; i <= 4; i++) {
                    double offset = skireil::utils::gaussian_rand(0, sigma);
                    offset = std::max(-sigma, std::min(offset, sigma));
                    engine_pose[i] = engine_pose[i] + offset;
                }
            }
            auto engine = robot_dart::Robot::create_box({0.38, 0.3, 0.3}, engine_pose, "fixed", 100., dart::Color::Red(1.0), "engine");
            simu.add_robot(engine);

        #ifdef GRAPHIC
            robot_dart::gui::magnum::GraphicsConfiguration configuration;
            configuration.width = 1280;
            configuration.height = 960;
            configuration.bg_color = Eigen::Vector4d{1.0, 1.0, 1.0, 1.0};
            simu.set_graphics(std::make_shared<robot_dart::gui::magnum::Graphics>(configuration));
            Eigen::Vector3d camera_pos = Eigen::Vector3d(-3, 0.0, 1.4);
            Eigen::Vector3d look_at = Eigen::Vector3d(0., 0.0, 1.);
            Eigen::Vector3d up = Eigen::Vector3d(0., 0., 1.);
            std::static_pointer_cast<robot_dart::gui::magnum::Graphics>(simu.graphics())->look_at(camera_pos, look_at, up);
        #endif
        }

        template <typename RolloutInfo, typename Params>
        void scene_peg(robot_dart::RobotDARTSimu& simu, const RolloutInfo& info)
        {
            // Add collision detector that works with meshes
            simu.world()->getConstraintSolver()->setCollisionDetector( dart::collision::FCLCollisionDetector::create());

            simu.world()->setGravity(Eigen::Vector3d(0, 0, 0));

            // Add goal marker
            Eigen::Vector6d goal_pose = Eigen::Vector6d::Zero();
            goal_pose.tail(3) = info.goal;
            // dims, pose, type, mass, color, name
            auto ellipsoid = robot_dart::Robot::create_ellipsoid({0.03, 0.03, 0.03}, goal_pose, "fixed", 1., dart::Color::Green(1.0), "goal_marker");
            // remove collisions from goal marker
            ellipsoid->skeleton()->getRootBodyNode()->setCollidable(false);
            // add ellipsoid to simu
            simu.add_robot(ellipsoid);

            // Insert table
            Eigen::Vector6d table_pose;
            table_pose << 0., 0., 0., 0.8, 0.4, 0.415;
            auto table = robot_dart::Robot::create_box({table_pose[3]*2, table_pose[4]*2, table_pose[5]*2}, table_pose, "fixed", 100., dart::Color::White(1.0), "table");
            simu.add_robot(table);

            // Insert workbox
            const double plastic_friction{0.3};
            Eigen::Vector6d workbox_pose;
            workbox_pose << 0., 0., 0., -(0.942/2.0), 0.48, 0.3;
            auto workbox = robot_dart::Robot::create_box({std::abs(workbox_pose[3]*2), 2.0, std::abs(workbox_pose[5]*2)}, workbox_pose, "fixed", 100., dart::Color::Gray(1.0), "workbox");
            workbox->set_friction_coeff("workbox", plastic_friction);
            workbox->skeleton()->getRootBodyNode()->setCollidable(false);
            simu.add_robot(workbox);

            // Insert block with hole
            std::vector<std::pair<std::string, std::string>> packages;
            packages.push_back(std::make_pair(std::string("skireil"), std::string(RESPATH)));
            auto box_with_hole = std::make_shared<robot_dart::Robot>(std::string(RESPATH) + "/URDF/box_with_hole.urdf", packages, "box_with_hole");
            box_with_hole->set_friction_coeff("box_link", plastic_friction);
            // Set box position
            Eigen::Vector3d box_position = Eigen::Vector3d(-0.675, -0.075, 0.603);
            if (Params::skireil::domain_randomization()) {
                // Randomly add offset in x and y direction
                double sigma{0.008};
                for (size_t i = 0; i <= 1; i++) {
                    double offset = skireil::utils::gaussian_rand(0, sigma);
                    offset = std::max(-sigma, std::min(offset, sigma));
                    box_position[i] = box_position[i] + offset;
                }
            }
            Eigen::Isometry3d T;
            T.linear() = dart::math::eulerXYZToMatrix(Eigen::Vector3d::Zero());
            T.translation() = box_position;
            auto joint = box_with_hole->skeleton()->getJoint("box_joint");
            joint->setTransformFromParentBodyNode(T);
            simu.add_robot(box_with_hole);

        #ifdef GRAPHIC
            robot_dart::gui::magnum::GraphicsConfiguration configuration;
            configuration.width = 1280;
            configuration.height = 960;
            configuration.bg_color = Eigen::Vector4d{1.0, 1.0, 1.0, 1.0};
            simu.set_graphics(std::make_shared<robot_dart::gui::magnum::Graphics>(configuration));
            Eigen::Vector3d camera_pos = Eigen::Vector3d(-3, 0.0, 1.4);
            Eigen::Vector3d look_at = Eigen::Vector3d(0., 0.0, 1.);
            Eigen::Vector3d up = Eigen::Vector3d(0., 0., 1.);
            std::static_pointer_cast<robot_dart::gui::magnum::Graphics>(simu.graphics())->look_at(camera_pos, look_at, up);
        #endif
        }

        template <typename RolloutInfo, typename Params>
        void scene_polyhedron(robot_dart::RobotDARTSimu& simu, const RolloutInfo& info)
        {
            // Add collision detector that works with meshes
            simu.world()->getConstraintSolver()->setCollisionDetector( dart::collision::FCLCollisionDetector::create());

            simu.world()->setGravity(Eigen::Vector3d(0, 0, -9.81));

            // Insert table
            Eigen::Vector6d table_pose;
            table_pose << 0., 0., 0., 0.8, 0.4, 0.415;
            auto table = robot_dart::Robot::create_box({table_pose[3]*2, table_pose[4]*2, table_pose[5]*2}, table_pose, "fixed", 100., dart::Color::White(1.0), "table");
            simu.add_robot(table);

            const double plastic_friction{0.25};
            // Insert workbox
            Eigen::Vector6d workbox_pose;
            workbox_pose << 0., 0., 0., -(0.942/2.0), 0.48, 0.3;
            auto workbox = robot_dart::Robot::create_box({std::abs(workbox_pose[3]*2), 2.0, std::abs(workbox_pose[5]*2)}, workbox_pose, "fixed", 100., dart::Color::Gray(1.0), "workbox");
            workbox->set_friction_coeff("workbox", plastic_friction);
            simu.add_robot(workbox);

            // Insert polyhedron bay and block with hole
            std::vector<std::pair<std::string, std::string>> packages;
            packages.push_back(std::make_pair(std::string("skireil"), std::string(RESPATH)));
            auto polyhedron_bay = std::make_shared<robot_dart::Robot>(std::string(RESPATH) + "/URDF/polyhedron_bay.urdf", packages, "polyhedron_bay");
            polyhedron_bay->set_friction_coeff("polyhedron_bay_link", 0.2);
            auto box_with_hole = std::make_shared<robot_dart::Robot>(std::string(RESPATH) + "/URDF/box_with_hole.urdf", packages, "box_with_hole");
            box_with_hole->set_friction_coeff("box_link", 0.2);
            box_with_hole->skeleton()->getRootBodyNode()->setCollidable(true);
            // Set polyhedron_bay position
            auto joint_box = box_with_hole->skeleton()->getJoint("box_joint");
            auto joint = polyhedron_bay->skeleton()->getJoint("polyhedron_bay_joint");
            Eigen::Isometry3d T = joint->getTransformFromChildBodyNode();
            Eigen::Isometry3d T_box = joint_box->getTransformFromParentBodyNode();
            Eigen::Vector3d bay_position = T.translation();
            Eigen::Vector3d box_position = T_box.translation();
            if (Params::skireil::domain_randomization())
            {
                // Randomly add offset in x and y direction
                double sigma{0.007};
                for (size_t i = 0; i <= 1; i++) {
                    double offset = skireil::utils::gaussian_rand(0, sigma);
                    offset = std::max(-sigma, std::min(offset, sigma));
                    bay_position[i] = bay_position[i] + offset;
                    box_position[i] = box_position[i] + offset;
                }
            }
            T.translation() = bay_position;
            T_box.translation() = box_position;
            joint->setTransformFromChildBodyNode(T);
            joint_box->setTransformFromParentBodyNode(T_box);
            simu.add_robot(polyhedron_bay);
            simu.add_robot(box_with_hole);

        #ifdef GRAPHIC
            robot_dart::gui::magnum::GraphicsConfiguration configuration;
            configuration.width = 1280;
            configuration.height = 960;
            configuration.bg_color = Eigen::Vector4d{1.0, 1.0, 1.0, 1.0};
            simu.set_graphics(std::make_shared<robot_dart::gui::magnum::Graphics>(configuration));
            Eigen::Vector3d camera_pos = Eigen::Vector3d(-3, 0.0, 1.4);
            Eigen::Vector3d look_at = Eigen::Vector3d(0., 0.0, 1.);
            Eigen::Vector3d up = Eigen::Vector3d(0., 0., 1.);
            std::static_pointer_cast<robot_dart::gui::magnum::Graphics>(simu.graphics())->look_at(camera_pos, look_at, up);
        #endif
        }

    } // namespace scenes
} // namespace learning_skills

#endif