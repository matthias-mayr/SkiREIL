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
#ifndef SKIREL_FOLDER_ACCESS
#define SKIREIL_FOLDER_ACCESS

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <iostream>

namespace fs = boost::filesystem;

std::string get_experiment_folder(std::string exp_folder = "/tmp/skireil/learning_skills/") {
    std::cout << "\nChecking for log files in: " << exp_folder << std::endl;
    std::vector<boost::filesystem::directory_entry> experiments;
    boost::filesystem::directory_iterator exp_dir_it{exp_folder};
    int ignored{0};
    int folder_per_line{3};
    for (int i = 0; exp_dir_it != boost::filesystem::directory_iterator(); exp_dir_it++) {
        std::string dir_name {exp_dir_it->path().filename().c_str()};
        if (dir_name.find("_replay") == std::string::npos) {
            std::cout << i << ") " << dir_name;
            if ((i+1) % folder_per_line) {
                std::cout << "\t";
            } else {
                std::cout << std::endl;
            }
            experiments.push_back(*exp_dir_it);
            i++;
        } else {
            ignored++;
        }
    }
    std::cout << "\nIgnored " << ignored << " folders." << std::endl;
    std::cout << "Choose an experiment to load by number. Press [enter] to load the latest." << std::endl;
    std::string exp_choice;
    std::getline( std::cin, exp_choice);
    if (exp_choice.empty()) {
        exp_folder = experiments.back().path().c_str();
    } else {
        exp_folder = experiments[std::stoi(exp_choice)].path().c_str();
    }
    return exp_folder;
}

std::string get_model_folder(std::string exp_folder) {
    boost::filesystem::directory_iterator model_dir_it{exp_folder + "/models"};
    std::vector<boost::filesystem::directory_entry> model_dirs;
    std::cout << "Choose a GP model to load:" << std::endl;
    std::cout << "0) No model"  << std::endl;
    for (int i = 1; model_dir_it != boost::filesystem::directory_iterator(); model_dir_it++) {
        std::string dir_name {model_dir_it->path().filename().c_str()};
        if (dir_name.find("model_learn") != std::string::npos) {
            std::cout << i << ") " << dir_name << std::endl;
            model_dirs.push_back(*model_dir_it);
            i++;
        }
    }
    std::string model_folder;
    if (model_dirs.empty()) {
        std::cout << "\nNo model found in this directory. Continue with parameters.\n" << std::endl;
    } else {
        std::cout << "Choose a model iteration to load by number. Press [enter] to load the latest." << std::endl;
        std::string model_choice;
        std::getline( std::cin, model_choice);
        if (model_choice.empty()) {
            model_folder = model_dirs.back().path().c_str();
            std::cout << "Model folder: " << model_folder << std::endl;
        } else if (std::stoi(model_choice) > 0) {
            model_folder = model_dirs[std::stoi(model_choice)-1].path().c_str();
            std::cout << "Model folder: " << model_folder << std::endl;
        } else {
            std::cout << "Not loading any model." << std::endl;
        }
    }
    return model_folder;
}

std::string get_param_file(std::string exp_folder) {
    boost::filesystem::directory_iterator param_dir_it{exp_folder+"/parameters"};
    std::vector<boost::filesystem::directory_entry> param_files;
    for (int i = 0; param_dir_it != boost::filesystem::directory_iterator(); param_dir_it++) {
        std::string param_name{param_dir_it->path().filename().c_str()};
        if (param_name.find(".json") != std::string::npos) {
            std::cout << i << ") " << param_name << std::endl;
            param_files.push_back(*param_dir_it);
            i++;
        }
    }
    if (param_files.empty()) {
        std::cout << "No parameter files found. Exiting.";
        return std::string();
    }
    std::cout << "Choose a parameter file to load by number. Press [enter] to load the latest." << std::endl;
    std::string param_filename;
    std::string param_choice;
    std::getline( std::cin, param_choice);
    if (param_choice.empty()) {
        param_filename = param_files.back().path().c_str();
    } else {
        param_filename = param_files[std::stoi(param_choice)].path().c_str();
    }
    std::cout << "Parameter file: " << param_filename << std::endl;
    return param_filename;
}

std::string get_trajectory_file(std::string exp_folder) {
    boost::filesystem::directory_iterator traj_dir_it{exp_folder+"/trajectories"};
    std::vector<boost::filesystem::directory_entry> traj_files;
    for (int i = 0; traj_dir_it != boost::filesystem::directory_iterator(); traj_dir_it++) {
        std::string traj_name{traj_dir_it->path().filename().c_str()};
        if (traj_name.find("_real") != std::string::npos) {
            std::cout << i << ") " << traj_name << std::endl;
            traj_files.push_back(*traj_dir_it);
            i++;
        }
    }
    std::cout << "Choose a trajectory file to load by number. Press [enter] to load the latest." << std::endl;
    std::string traj_filename;
    std::string param_choice;
    std::getline( std::cin, param_choice);
    if (param_choice.empty()) {
        traj_filename = traj_files.back().path().c_str();
    } else {
        traj_filename = traj_files[std::stoi(param_choice)].path().c_str();
    }
    std::cout << "Trajectory file: " << traj_filename << std::endl;
    return traj_filename;
}

void copyDirectoryRecursively(const fs::path& sourceDir, const fs::path& destinationDir)
{
    if (!fs::exists(sourceDir) || !fs::is_directory(sourceDir))
    {
        throw std::runtime_error("Source directory " + sourceDir.string() + " does not exist or is not a directory");
    }
    if (fs::exists(destinationDir))
    {
        throw std::runtime_error("Destination directory " + destinationDir.string() + " already exists");
    }
    if (!fs::create_directory(destinationDir))
    {
        throw std::runtime_error("Cannot create destination directory " + destinationDir.string());
    }

    for (const auto& dirEnt : fs::recursive_directory_iterator{sourceDir})
    {
        const auto& path = dirEnt.path();
        auto relativePathStr = path.string();
        boost::replace_first(relativePathStr, sourceDir.string(), "");
        fs::copy(path, destinationDir / relativePathStr);
    }
}

#endif