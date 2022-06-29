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

using namespace learning_skills;

template <typename Params>
int get_iteration() {
    int iteration{0};
    // Get parameter files
    std::vector<std::string> param_files;
    boost::filesystem::recursive_directory_iterator param_it(Params::meta_conf::params_folder());
    boost::filesystem::recursive_directory_iterator endit;
    while (param_it != endit) {
        if(boost::filesystem::is_regular_file(*param_it) && (param_it->path().string().find("_opt_final.bin") != std::string::npos)) {
            iteration++;
        }
        ++param_it;
    }
    return iteration;
}

template <typename Params, typename T>
int read_existing_data(T& system) {
    LOG(INFO) << "Continuing from existing experiment folder.";
    boost::filesystem::recursive_directory_iterator endit;
    // Get trajectory files
    std::vector<std::string> traj_files;
    boost::filesystem::recursive_directory_iterator traj_it(Params::meta_conf::traj_folder());
    while (traj_it != endit) {
        if(boost::filesystem::is_regular_file(*traj_it) && (traj_it->path().string().find("traj_real_") != std::string::npos)) {
            traj_files.push_back(traj_it->path().string());
        }
        ++traj_it;
    }
    // Get parameter files
    std::vector<std::string> param_files;
    boost::filesystem::recursive_directory_iterator param_it(Params::meta_conf::params_folder());
    while (param_it != endit) {
        if(boost::filesystem::is_regular_file(*param_it) && (param_it->path().string().find("_opt_final.json") != std::string::npos)) {
            param_files.push_back(param_it->path().string());
        }
        ++param_it;
    }
    std::sort(traj_files.begin(), traj_files.end());
    std::sort(param_files.begin(), param_files.end());

    ParamVec params;
    for (size_t i = 0; i < param_files.size()-1; i++) {
        LOG(INFO) << "Loading trajectory from: " << traj_files[i];
        LOG(INFO) << "Loading parameters from: " << param_files[i];
        params = skireil::parameters::paramVecFromJson(skireil::parameters::fileToJson(param_files[i]));
        LOG(INFO) << "Parameter: " << skireil::parameters::paramVecToStream(params, true).rdbuf();
        system.populate_observations(traj_files[i], params);
    }
    // Set policy parameters
    params = skireil::parameters::paramVecFromJson(skireil::parameters::fileToJson(param_files.back()));
    system.set_policy_params(params);

    LOG(INFO) << "Restarting learning from iteration " << param_files.size();
    return param_files.size();
}

int main(int argc, char** argv)
{
    if (!process_configuration(argc, argv)) {
        std::cerr << "Could not process the configuration. Exiting." << std::endl;
        return -1;
    }

    if (!init_folder("/tmp/skireil/learning_skills/")) {
        std::cerr << "Error when creating experiment folder. Exiting." << std::endl;
        return -2;
    }
    boost::filesystem::copy_file(Params::meta_conf::config_file(), Params::meta_conf::folder()+"scenario.json");

    init_logging(argv);
    write_parameters_to_log();

    using policy_t = skireil::policy::SkiROSPolicy<PolicyParams>;

    using kernel_t = limbo::kernel::SquaredExpARD<Params>;
    using mean_t = limbo::mean::Constant<Params>;

    using GP_t = limbo::model::MultiGP<Params, limbo::model::GP, kernel_t, mean_t, limbo::model::multi_gp::ParallelLFOpt<Params, skireil::model::gp::KernelLFOpt<Params>>>;

    using MGP_t = skireil::model::GPModel<Params, GP_t>;

    using rolloutinfo_t = RolloutInfo;

    using reward_t = std::multimap<std::string, std::shared_ptr<skireil::reward::Reward<rolloutinfo_t>>>;

    ParamVec pv = paramVecFromJson(Params::skireil::optimizer_config()["input_parameters"]);
    Params::skireil::set_param_vec(pv);

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

    // ROS
    ros::init(argc, argv, "skireil", ros::init_options::AnonymousName);

    // Start learning
    //
    // Arg 1: Iteration to start from
    // Arg 2: Learning iterations
    // Arg 3: Random policies
    int start_it = 0;
    int iterations = 0;
    LOG(INFO) << "Leaving 'main', entering 'learn' function.";
    if (Params::skireil::optimizer() == "cma_es") {
        // TODO: Param fix. Adapt CMA-ES.
        // using policy_opt_t = limbo::opt::Cmaes<Params>;
        // skireil::BlackDROPS<Params, MGP_t, SimpleArm, policy_t, policy_opt_t, RewardFunction> arm_system;
        // if (!Params::meta_conf::existing_data_folder().empty()) {
        //     start_it = read_existing_data<Params,skireil::BlackDROPS<Params, MGP_t, SimpleArm, policy_t, policy_opt_t, RewardFunction>> (arm_system);
        // }
        // arm_system.learn(start_it, iterations, true);
    } else {
        using policy_opt_t = skireil::opt::Hypermapper<Params>;
        start_it = get_iteration<Params>();
        Params::opt_hm::set_start_it(start_it);
        using system_t = skireil::BlackDROPS<Params, MGP_t, SimpleArm, policy_t, policy_opt_t, reward_t, rolloutinfo_t>;

        system_t arm_system;
        arm_system.set_reward_functions(reward_functions);
        arm_system.set_scene_setup(get_scene<rolloutinfo_t, Params>(Params::skireil::scene()));
        if (!Params::meta_conf::existing_data_folder().empty()) {
            start_it = read_existing_data<Params,system_t> (arm_system);
        }
        arm_system.learn(start_it, iterations, true);
    }
    // Place DONE file as an easy indicator for a finished experiment
    boost::filesystem::ofstream(Params::meta_conf::folder() + "DONE");
    return 0;
}
