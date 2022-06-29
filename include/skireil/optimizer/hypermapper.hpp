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
#ifndef SKIREIL_OPT_HM_HPP
#define SKIREIL_OPT_HM_HPP

#include <skireil/parameters.hpp>
#include <skireil/utils/json.hpp>
#include <skireil/utils/utils.hpp>

#ifdef USE_TBB
#include <tbb/tbb.h>
#endif

using json = nlohmann::json;
using namespace skireil::parameters;
using ParamVec = std::vector<std::shared_ptr<ParamBase>>;

namespace skireil {

namespace opt {

    /// parallel for
    template <typename F>
    inline void loop(size_t begin, size_t end, const F& f)
    {
#ifdef USE_TBB
        tbb::parallel_for(size_t(begin), end, size_t(1), [&](size_t i) {
            // clang-format off
        f(i);
            // clang-format on
        });
#else
        for (size_t i = begin; i < end; ++i)
            f(i);
#endif
    }

    // HyperMapper Objective object
    struct HMObjective {
        float reward;
    };

    template <typename Params>
    struct Hypermapper {
    public:
        Hypermapper() {
            if (!std::getenv("HYPERMAPPER_HOME") || !std::getenv("PYTHONPATH")) {
                std::string ErrMsg = "Environment variables are not set!\n";
                ErrMsg += "Please set HYPERMAPPER_HOME and PYTHONPATH before running this ";
                fatalError(ErrMsg);
            }
            _opt_nr = Params::opt_hm::start_it();
            _log_folder = Params::meta_conf::opt_folder();
        }

        template <typename F>
        ParamVec operator()(const F& f, const ParamVec& init, double bounded)
        {
            // Create json scenario
            std::string JSonFileNameStr =
            createjson(Params::meta_conf::application_name(), _log_folder, Params::skireil::max_fun_evals(),
            false, init, Params::skireil::objectives());

            // Launch HyperMapper
            std::string cmd("python3 ");
            cmd += getenv("HYPERMAPPER_HOME");
            cmd += "/hypermapper/optimizer.py";
            cmd += " " + JSonFileNameStr;

            LOG(INFO) << "Executing command: " << cmd << std::endl;
            struct skireil::utils::popen2 hypermapper;
            skireil::utils::popen2(cmd.c_str(), &hypermapper);

            ParamVec res = opt(hypermapper, init, f);
            _opt_nr++;
            return res;
        }

    private:
        int _opt_nr{0};
        std::string _log_folder;
        bool _feasibility{false};

        template <typename F>
        ParamVec opt(struct skireil::utils::popen2& hypermapper, const ParamVec& InParams, const F& f) {
            FILE *instream = fdopen(hypermapper.from_child, "r");
            FILE *outstream = fdopen(hypermapper.to_child, "w");
            const int max_buffer = 1000;
            char buffer[max_buffer];
            char* res;
            size_t numParams{InParams.size()};
            ParamVec params;
            double best_reward{-INFINITY};
            // Loop that communicates with HyperMapper
            int i = 0;
            while (true) {
                res = fgets(buffer, max_buffer, instream);
                if (res == NULL) {
                    fatalError("'fgets' reported an error.");
                }
                if (Params::meta_conf::verbose()) {
                    LOG(INFO) << "Iteration: " << i << std::endl;
                    LOG(INFO) << "Recieved: " << buffer;
                }
                // Receiving Num Requests
                std::string bufferStr(buffer);
                if (!bufferStr.compare("End of HyperMapper\n")) {
                    LOG(INFO) << "Hypermapper completed!\n";
                    break;
                }
                std::string NumReqStr = bufferStr.substr(bufferStr.find(' ') + 1);
                int numRequests = 0;
                try
                {
                    numRequests = stoi(NumReqStr);
                }
                catch (const std::invalid_argument &e)
                {
                    LOG(FATAL) << "Hypermapper failed. Check the logs.";
                }
                // Receiving input param names
                res = fgets(buffer, max_buffer, instream);
                if (res == NULL) {
                    fatalError("'fgets' reported an error.");
                }
                bufferStr = std::string(buffer);
                if (Params::meta_conf::verbose()) {
                    LOG(INFO) << "Recieved: " << buffer;
                }
                // Work out response
                std::string response;
                std::vector<std::string> responseArray(numRequests);
                std::vector<std::vector<double>> rewards(numRequests);
                std::vector<ParamVec> params_requests(numRequests);
                // Create mapping for InputParam objects to keep track of order
                // Also check if parameters exist and starts constructing the response
                std::map<int, int> InputParamsMap;
                size_t pos = 0;
                for (size_t param = 0; param < numParams; param++) {
                    size_t len = bufferStr.find_first_of(",\n", pos) - pos;
                    std::string ParamStr = bufferStr.substr(pos, len);
                    //      std::cout << "  -- param: " << ParamStr << "\n";
                    auto paramIt = findParamByKey(InParams, ParamStr);
                    if (paramIt >= 0) {
                        InputParamsMap[param] = paramIt;
                        response += ParamStr;
                        response += ",";
                    } else {
                        fatalError("Unknown parameter received: " + ParamStr);
                    }
                    pos = bufferStr.find_first_of(",\n", pos) + 1;
                }
                for (const auto& objString : Params::skireil::objectives()) {
                    response += objString + ",";
                }
                if (_feasibility) {
                    response += "feasible,";
                }
                response += "\n";
                // Fetch parameters and save them to a ParamVec and string
                for (int request = 0; request < numRequests; request++) {
                    ParamVec np = cloneParamVec(InParams);

                    std::string r_line;
                    // Receiving paramter values
                    res = fgets(buffer, max_buffer, instream);
                    if (res == NULL) {
                        fatalError("'fgets' reported an error.");
                    }
                    if (Params::meta_conf::verbose()) {
                        LOG(INFO) << "Received: " << buffer;
                    }
                    bufferStr = std::string(buffer);
                    pos = 0;
                    for (size_t param = 0; param < numParams; param++) {
                        size_t len = bufferStr.find_first_of(",\n", pos) - pos;
                        std::string ParamValStr = bufferStr.substr(pos, len);
                        setInputValue(np[InputParamsMap[param]].get(), ParamValStr);
                        r_line += ParamValStr;
                        r_line += ",";
                        pos = bufferStr.find_first_of(",\n", pos) + 1;
                    }
                    params_requests[request] = np;
                    responseArray[request] = r_line;
                }
                // Evaluate the parameters
                loop(0, numRequests, [&](size_t request) {
                    reward::RewardResultMap res = f(params_requests[request], false);
                    std::vector<double> rews;
                    for (const auto& objString : Params::skireil::objectives()) {
                        rews.push_back(reward::get_reward_by_objective(res, objString));
                    }
                    rewards[request] = rews;
                    // Save result into response string
                    // HyperMapper wants to minimize a function
                    for (const auto& obj : rewards[request]) {
                        responseArray[request] += std::to_string(-obj);
                        responseArray[request] += ",";
                    }
                    if (_feasibility) {
                        if (reward::check_feasibility(res)) {
                            responseArray[request] += "True,";
                        } else {
                            responseArray[request] += "False,";
                        }
                    }
                    responseArray[request] += "\n";
                });
                // Determine the best parameters and construct response
                for (int request = 0; request < numRequests; request++) {
                    // If we are in a single objective setting, we can save the best result
                    if (rewards[request].size() == 1 && rewards[request][0] > best_reward) {
                            best_reward = rewards[request][0];
                            params = params_requests[request];
                    }
                    response += responseArray[request];
                }
                if (Params::meta_conf::verbose()) {
                    LOG(INFO) << "Response:\n" << response;
                }
                fputs(response.c_str(), outstream);
                fflush(outstream);
                i++;
            }

            close(hypermapper.from_child);
            close(hypermapper.to_child);
            return params;
        }

        // Function that creates the json scenario for hypermapper
        // Arguments:
        // - AppName: Name of application
        // - OutputFolderName: Name of output folder
        // - NumEvaluations: Number of evaluations
        // - NumDSERandomSamples: Number of HP random samples
        // - Predictor: Boolean for enabling/disabling feasibility predictor
        // - InParams: vector of input parameters
        // - Objectives: string with objective names
        std::string createjson(std::string AppName, std::string OutputFoldername, int NumEvaluations, bool Predictor,
                        const std::vector<std::shared_ptr<ParamBase>> &InParams, std::vector<std::string> Objectives) {
        int NumDSERandomSamples = 0;
        json HMScenario;
        if (!Params::skireil::optimizer_config().empty()) {
            HMScenario = Params::skireil::optimizer_config();
            if (HMScenario.contains("design_of_experiment")) {
                if (HMScenario["design_of_experiment"].contains("number_of_samples")) {
                    NumDSERandomSamples = HMScenario["design_of_experiment"]["number_of_samples"];
                }
            }
        }
        HMScenario["application_name"] = AppName;
        HMScenario["optimization_objectives"] = json(Objectives);
        HMScenario["hypermapper_mode"]["mode"] = "client-server";
        HMScenario["run_directory"] = Params::meta_conf::folder();
        HMScenario["log_file"] = OutputFoldername + "log_" + std::to_string(_opt_nr) + ".log";
        if (!HMScenario.contains("evaluations_per_optimization_iteration")) {
            // Black-DROPS undestanding of "iterations" is the number of total evaluations
            // For hypermapper it is iterations*evaluations
            int eval_per_it = Params::meta_conf::threads();
            if (eval_per_it < 0) {
                // TBB does not allow to access the number of threads:
                // https://stackoverflow.com/a/16138486
                // The VMs have 4 cores. 
                eval_per_it = 4;
            }
            LOG(INFO) << "No hypermapper batch size specified defaulting to " << eval_per_it << ".";
            HMScenario["evaluations_per_optimization_iteration"] = eval_per_it;
            // Determine number of iterations
            int hm_iterations{0};
            if (NumEvaluations - NumDSERandomSamples > 0) {
                hm_iterations = (NumEvaluations - NumDSERandomSamples)/eval_per_it;
                // If we have a remainder, add one to ensure we reach at least the desired number.
                if ((NumEvaluations - NumDSERandomSamples) % eval_per_it != 0) {
                    hm_iterations++;
                }
            } else {
                std::cout << "Number of DSE random samples is larger than the number of evaluations." << std::endl;
            }
            HMScenario["optimization_iterations"] = hm_iterations;
        } else {
            HMScenario["evaluations_per_optimization_iteration"] = 1;
            HMScenario["optimization_iterations"] = NumEvaluations - NumDSERandomSamples;
        }
        if (!HMScenario["models"].contains("model")) {
            HMScenario["models"]["model"] = "gaussian_process";
        }
        if (Predictor) {
            HMScenario["feasible_output"]["enable_feasible_predictor"] = true;
        }
        if (HMScenario.contains("feasible_output")) {
            HMScenario["feasible_output"]["name"] = "feasible";
            HMScenario["feasible_output"]["false_value"] = "False";
            HMScenario["feasible_output"]["true_value"] = "True";
        }
        if (HMScenario["feasible_output"]["enable_feasible_predictor"].get<bool>()) {
            _feasibility = true;
        }

        HMScenario["output_data_file"] =
            OutputFoldername + "/" + AppName + "_output_data.csv";
        HMScenario["output_pareto_file"] =
            OutputFoldername + "/" + AppName + "_output_pareto.csv";
        HMScenario["output_image"]["output_image_pdf_file"] =
            OutputFoldername + "_" + AppName + "_output_image.pdf";

        if (!HMScenario.contains("design_of_experiment")) {
            json HMDOE;
            HMDOE["doe_type"] = "random sampling"; // "random sampling";
            HMDOE["number_of_samples"] = NumDSERandomSamples;
            HMScenario["design_of_experiment"] = HMDOE;
        }
        if (HMScenario["design_of_experiment"]["number_of_samples"].get<int>() > NumEvaluations) {
            LOG(WARNING) << "More DoE samples than number of iterations: " << HMScenario["design_of_experiment"]["number_of_samples"].get<int>() << " > " << NumEvaluations <<
            ". Limiting DoE samples.";
            HMScenario["design_of_experiment"]["number_of_samples"] = NumEvaluations;
        }

        //  cout << setw(4) << HMScenario << endl;
        std::ofstream HyperMapperScenarioFile;

        std::string JSonFileNameStr =
            OutputFoldername + "scenario_" + std::to_string(_opt_nr) + ".json";

        HyperMapperScenarioFile.open(JSonFileNameStr);
        if (HyperMapperScenarioFile.fail()) {
            fatalError("Unable to open file: " + JSonFileNameStr);
        }
        LOG(INFO) << "Writing JSON file to: " << JSonFileNameStr << std::endl;
        HyperMapperScenarioFile << std::setw(4) << HMScenario << std::endl;
        return JSonFileNameStr;
        }
    };

    } // namespace opt
} // namespace skireil

#endif