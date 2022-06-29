#!/usr/bin/env python3
# encoding: utf-8
#|
#|    Copyright Inria July 2017
#|    This project has received funding from the European Research Council (ERC) under
#|    the European Union's Horizon 2020 research and innovation programme (grant
#|    agreement No 637972) - see http://www.resibots.eu
#|
#|    Contributor(s):
#|      - Matthias Mayr (matthias.mayr@cs.lth.se)
#|      - Konstantinos Chatzilygeroudis (konstantinos.chatzilygeroudis@inria.fr)
#|      - Rituraj Kaushik (rituraj.kaushik@inria.fr)
#|      - Roberto Rama (bertoski@gmail.com)
#|
#|
#|    This software is governed by the CeCILL-C license under French law and
#|    abiding by the rules of distribution of free software.  You can  use,
#|    modify and/ or redistribute the software under the terms of the CeCILL-C
#|    license as circulated by CEA, CNRS and INRIA at the following URL
#|    "http://www.cecill.info".
#|
#|    As a counterpart to the access to the source code and  rights to copy,
#|    modify and redistribute granted by the license, users are provided only
#|    with a limited warranty  and the software's author,  the holder of the
#|    economic rights,  and the successive licensors  have only  limited
#|    liability.
#|
#|    In this respect, the user's attention is drawn to the risks associated
#|    with loading,  using,  modifying and/or developing or reproducing the
#|    software by the user in light of its specific status of free software,
#|    that may mean  that it is complicated to manipulate,  and  that  also
#|    therefore means  that it is reserved for developers  and  experienced
#|    professionals having in-depth computer knowledge. Users are therefore
#|    encouraged to load and test the software's suitability as regards their
#|    requirements in conditions enabling the security of their systems and/or
#|    data to be ensured and,  more generally, to use and operate it in the
#|    same conditions as regards security.
#|
#|    The fact that you are presently reading this means that you have had
#|    knowledge of the CeCILL-C license and that you accept its terms.
#|
import sys, getopt
import os
import json
import pandas as pd
import argparse

def get_immediate_subdirectories(a_dir):
    return [name for name in os.listdir(a_dir)
            if os.path.isdir(os.path.join(a_dir, name))]

def create_json(parameters, params_list):
    scenario = {}
    for param in params_list:
        scenario[param] = {}
        scenario[param]["parameter_default"] = parameters.loc[param]
        scenario[param]["parameter_type"] = "real"
        scenario[param]["values"] = [parameters.loc[param],parameters.loc[param]]
    return scenario

def get_experiment_name(folder):
    f = open(folder + "scenario.json")
    scenario = json.loads(f.read())
    return scenario["application_name"]

def read_write_json(output_file, params, params_list):
    with open(output_file, "w") as scenario_file:
        json.dump(create_json(params,params_list), scenario_file, indent=4)  

def compute_pareto(dir):
    input_json = dir + "optimizations/scenario_0.json"
    os.system("python3 -m hypermapper.compute_pareto " + input_json)

def plot_pareto(dir):
    input_json = dir + "optimizations/scenario_0.json"
    os.system("python3 -m hypermapper.plot_pareto " + input_json)

def create_params_config(dir, experiment):
    file = "optimizations/ABC_output_pareto.csv"
    input_csv = dir + file.replace("ABC", experiment)
    f = open(dir + "optimizations/scenario_0.json")
    data = json.loads(f.read())
    params_list = list(data['input_parameters'].keys())
    objectives_list = list(data["optimization_objectives"])
    if "force" in objectives_list:
        sort_objective = "force"
    else:
        sort_objective = objectives_list[0]
    print("Sorting by objective", sort_objective)
    all_params = pd.read_csv(input_csv)
    all_params = all_params.sort_values(by=[sort_objective])
    count = 0
    for index in range(len(all_params.index)):
        params = all_params[params_list].iloc[index]
        read_write_json(dir + "parameters/policy_params_generated_" + str(index) + ".json",params, params_list)
        count += 1
    print (str(count) + " parameter configuration files created")

def main():
    _, args = getopt.getopt(sys.argv[1:], "ho:v", ["help", "output="])
    if not args or len(args) != 1:
        print("Error")
        print("Needs 1 argument, 'folder location'.")
        sys.exit(2)
    else:
        main_dir = args[0]
        if main_dir[-1] is not "/":
            main_dir += "/"
        print(main_dir)
        if main_dir == "/tmp/skireil/learning_skills/":
            folders = get_immediate_subdirectories(main_dir)
            for f in folders:
                exp_dir = main_dir + f + "/"
                print("Processing", get_experiment_name(exp_dir))
                compute_pareto(exp_dir)
                plot_pareto(exp_dir)
                create_params_config(exp_dir, get_experiment_name(exp_dir))
        else:
            main_dir = args[0]
            compute_pareto(main_dir)
            plot_pareto(main_dir)
            create_params_config(main_dir, get_experiment_name(main_dir))

if __name__ == "__main__":
    main()