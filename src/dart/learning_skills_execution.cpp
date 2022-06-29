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

#include <chrono>
#include <exception>

#include <skireil/utils/utils.hpp>
#include <skireil/utils/folder_access.hpp>
#include <skireil/experiments/learning_skills.hpp>
#include <skireil/experiments/learning_skills_execution.hpp>
#include <skireil/parameters.hpp>

using namespace learning_skills;
using namespace skireil;

int main(int argc, char** argv)
{
    skireil::utils::CmdArgs cmd_arguments;
    int ret = cmd_arguments.parse(argc, argv);
    if (ret >= 0)
        return ret;
    std::string exp_name, folder;
    folder = "/tmp/skireil_execution/learning_skills/";
    if (boost::filesystem::create_directories(folder)) {
        boost::filesystem::permissions(folder, boost::filesystem::add_perms|boost::filesystem::all_all);
    }

    if (!init_folder(folder, exp_name)) {
        std::cerr << "Error when creating experiment folder. Exiting." << std::endl;
        return -2;
    }

    init_logging(argv);

    // ROS
    ros::init(argc, argv, "skireil_execution", ros::init_options::AnonymousName);

    // Check for available folders
    std::string exp_folder;
    if (cmd_arguments.data().empty()) {
        exp_folder = get_experiment_folder();
    } else {
        exp_folder = cmd_arguments.data();
    }

    std::string param_filename = cmd_arguments.config();
    std::cout << "Experiment folder: " << exp_folder << std::endl;
    cmd_arguments.set_config(exp_folder + "/scenario.json");

    if (!process_configuration(cmd_arguments)) {
        std::cerr << "Could not process the configuration. Exiting." << std::endl;
        return -1;
    }
    write_parameters_to_log();

    Params::meta_conf::set_verbose(true);
    learning_skills_execution<PolicyControl<Params, global::policy_t>, Params> execution;
    LOG(INFO) << "Setting up simulation and other components.";
    init_simu(Params::skireil::robot_config());
    auto robot = global::global_robot->clone();
    if (param_filename.empty()) {
        param_filename = get_param_file(exp_folder);
    }
    LOG(INFO) << "Parameter file to load: " << param_filename;
    parameters::ParamVec params = parameters::paramVecFromJson(parameters::fileToJson(param_filename));
    boost::filesystem::copy_file(param_filename, Params::meta_conf::params_folder() + "params.json");
    std::shared_ptr<PolicyControl<Params, global::policy_t>> controller = execution.configure_controller(robot, params);
    // execution.set_params(get_parameters<Params>(experiment, parameter_file));

    std::string traj_sparse_filename(Params::meta_conf::traj_folder() + "traj_sparse.dat");
    std::string traj_dense_filename(Params::meta_conf::traj_folder() + "traj_dense.dat");
    std::string traj_dense_ee_filename(Params::meta_conf::traj_folder() + "traj_dense_ee.dat");
    std::string abort;
    while (abort.empty())
    {
        controller->configure(params);
        execution.run(controller, robot, !cmd_arguments.real_rollout());
        execution.publish_cartesian_wrench(Eigen::VectorXd::Zero(6));
        execution.save_data(controller, traj_sparse_filename, traj_dense_filename, traj_dense_ee_filename);
        if (cmd_arguments.real_rollout()) {
            std::cout << "\nPress [enter] to run again. Any other response exits." << std::endl;
            std::getline( std::cin, abort);
        } else {
            abort = "stop";
        }
    }
    return 0;
}
