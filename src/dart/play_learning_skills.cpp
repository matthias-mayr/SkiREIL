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
#include <skireil/experiments/learning_skills.hpp>
#include <skireil/utils/folder_access.hpp>

#include <boost/filesystem.hpp>

using namespace learning_skills;


int main(int argc, char** argv)
{
    skireil::utils::CmdArgs cmd_arguments;
    int ret = cmd_arguments.parse(argc, argv);
    if (ret >= 0)
        return ret;
    std::string exp_name, folder;
    if (cmd_arguments.data().empty()) {
        folder = "/tmp/skireil_replay/learning_skills/";
        if (boost::filesystem::create_directories(folder)) {
            boost::filesystem::permissions(folder, boost::filesystem::add_perms|boost::filesystem::all_all);
        }
    } else {
        folder = cmd_arguments.data() + "/replay/";
    }
    if (cmd_arguments.real_rollout()) {
        exp_name = "_trajectory/";
    } else {
        exp_name = "_params/";
    }

    if (!init_folder(folder, exp_name)) {
        std::cerr << "Error when creating experiment folder. Exiting." << std::endl;
        return -2;
    }

    init_logging(argv);

    using policy_t = skireil::policy::SkiROSPolicy<PolicyParams>;
    using policy_opt_t = skireil::opt::Hypermapper<Params>;

    using kernel_t = limbo::kernel::SquaredExpARD<Params>;
    using mean_t = limbo::mean::Constant<Params>;

    using GP_t = limbo::model::MultiGP<Params, limbo::model::GP, kernel_t, mean_t, limbo::model::multi_gp::ParallelLFOpt<Params, skireil::model::gp::KernelLFOpt<Params>>>;

    using MGP_t = skireil::model::GPModel<Params, GP_t>;

    using rolloutinfo_t = RolloutInfo;

    using reward_t = std::multimap<std::string, std::shared_ptr<skireil::reward::Reward<rolloutinfo_t>>>;

    ParamVec pv = paramVecFromJson(Params::skireil::optimizer_config()["input_parameters"]);
    Params::skireil::set_param_vec(pv);

    // ROS
    ros::init(argc, argv, "skireil_replay", ros::init_options::AnonymousName);

    // Check for available folders
    std::string exp_folder;
    if (cmd_arguments.data().empty()) {
        exp_folder = get_experiment_folder();
    } else {
        exp_folder = cmd_arguments.data();
    }
    std::cout << "Experiment folder: " << exp_folder << std::endl;
    cmd_arguments.set_config(exp_folder + "/scenario.json");

    if (!process_configuration(cmd_arguments)) {
        std::cerr << "Could not process the configuration. Exiting." << std::endl;
        return -1;
    }
    write_parameters_to_log();

    //
    // Set up simulation and other components
    //
    LOG(INFO) << "Setting up simulation and other components.";
    init_simu(Params::skireil::robot_config());

    // Register reward functions
    using namespace skireil::reward;
    std::map<std::string, std::shared_ptr<Reward<rolloutinfo_t>>(*)()> reward_function_map;
    reward_function_map["FixedSuccessReward"] = &createRewardFunction<skireil::reward::FixedSuccessReward<rolloutinfo_t>>;
    reward_function_map["GoalDistanceTranslationReward"] = &createRewardFunction<skireil::reward::GoalDistanceTranslationReward<rolloutinfo_t>>;
    reward_function_map["BoxAvoidanceReward"] = &createRewardFunction<skireil::reward::BoxAvoidanceReward<rolloutinfo_t>>;
    reward_function_map["LinearDistanceToBoxReward"] = &createRewardFunction<skireil::reward::LinearDistanceToBoxReward<rolloutinfo_t>>;
    reward_function_map["ForceApplicationReward"] = &createRewardFunction<skireil::reward::ForceApplicationReward<rolloutinfo_t>>;
    reward_function_map["ObjectPoseReward"] = &createRewardFunction<skireil::reward::ObjectPoseReward<rolloutinfo_t>>;
    reward_function_map["ObjectPositionReward"] = &createRewardFunction<skireil::reward::ObjectPositionReward<rolloutinfo_t>>;
    reward_function_map["ObjectOrientationReward"] = &createRewardFunction<skireil::reward::ObjectOrientationReward<rolloutinfo_t>>;
    reward_function_map["EndEffectorReferencePosition"] = &createRewardFunction<skireil::reward::EndEffectorReferencePosition<rolloutinfo_t>>;

    std::shared_ptr<reward_t> reward_functions = process_reward_config<reward_t>(Params::skireil::reward_config(), reward_function_map, global::global_robot);
    Params::skireil::set_objectives(skireil::utils::getKeys(*reward_functions));

    using system_t = skireil::SkiREIL<Params, MGP_t, SimpleArm, policy_t, policy_opt_t, reward_t, rolloutinfo_t>;
    system_t arm_system;
    arm_system.set_reward_functions(reward_functions);
    arm_system.set_scene_setup(get_scene<rolloutinfo_t, Params>(Params::skireil::scene()));

    std::string model_folder = get_model_folder(exp_folder);
    // Copy model folder
    if(!model_folder.empty()) {
        copyDirectoryRecursively(boost::filesystem::path(model_folder), boost::filesystem::path(Params::meta_conf::model_folder()));
    }
    std::string traj_sparse_filename = std::string(Params::meta_conf::traj_folder() + "traj_sparse.dat");
    std::string traj_dense_filename = std::string(Params::meta_conf::traj_folder() + "traj_dense.dat");
    std::string traj_dense_ee_filename = std::string(Params::meta_conf::traj_folder() + "traj_dense_ee.dat");
    std::string residual_filename = std::string(Params::meta_conf::traj_folder() + "residual.dat");
    std::string abort;
    // Replay parameter config
    if (!cmd_arguments.real_rollout()) {
        std::string param_filename = get_param_file(exp_folder);

        while(abort.empty()) {
            arm_system.play_iteration(model_folder, param_filename, traj_sparse_filename, traj_dense_filename, traj_dense_ee_filename, residual_filename);
            std::cout << "\nPress [enter] to run again. Any other response exits." << std::endl;
            std::getline( std::cin, abort);
        }
    // Replay trajectory
    } else {
        std::string traj_filename = get_trajectory_file(exp_folder);
        // Copy trajectory file
        boost::filesystem::copy_file(boost::filesystem::path(traj_filename), boost::filesystem::path(Params::meta_conf::traj_folder() + "traj_real.dat"));

        while(abort.empty()) {
            arm_system.replay_trajectory_with_model(model_folder, traj_filename, traj_sparse_filename, traj_dense_filename, traj_dense_ee_filename, residual_filename);
            std::cout << "\nPress [enter] to run again. Any other response exits." << std::endl;
            std::getline( std::cin, abort);
        }
    }
    return 0;
}