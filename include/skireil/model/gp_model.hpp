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
#ifndef SKIREIL_MODEL_GP_MODEL_HPP
#define SKIREIL_MODEL_GP_MODEL_HPP

#include <limbo/serialize/text_archive.hpp>

#include <skireil/model/base_model.hpp>

namespace skireil {
    namespace model {
        template <typename Params, typename GP_t>
        class GPModel : public BaseModel {
        public:
            GPModel() { init(); }

            void init()
            {
                _gp_model = GP_t(Params::skireil::model_input_dim() + Params::skireil::action_dim(), Params::skireil::model_pred_dim());
                _initialized = true;
            }

            void learn(const std::vector<std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd>>& observations)
            {
                std::vector<Eigen::VectorXd> samples, observs;
                Eigen::MatrixXd obs(observations.size(), std::get<2>(observations[0]).size());
                for (size_t i = 0; i < observations.size(); i++) {
                    Eigen::VectorXd st, act, pred;
                    st = std::get<0>(observations[i]);
                    act = std::get<1>(observations[i]);
                    pred = std::get<2>(observations[i]);

                    Eigen::VectorXd s(st.size() + act.size());
                    s.head(st.size()) = st;
                    s.tail(act.size()) = act;

                    samples.push_back(s);
                    obs.row(i) = pred;
                    observs.push_back(pred);
                    // std::cout << s.transpose() << std::endl;
                    // std::cout << pred.transpose() << std::endl;
                }

                std::cout << "GP Samples: " << samples.size() << std::endl;
                if (!_initialized)
                    init();

                _gp_model.compute(samples, observs, true);
                _gp_model.optimize_hyperparams();
            }

            std::tuple<Eigen::VectorXd, Eigen::VectorXd> predict(const Eigen::VectorXd& x, bool compute_variance = true) const
            {
                if (compute_variance) {
                   const auto& gp_models = _gp_model.gp_models();
                   Eigen::VectorXd mu = _gp_model.mu(x);
                   Eigen::VectorXd sigma(_gp_model.dim_out());

                   for (size_t i = 0; i < gp_models.size(); i++) sigma(i) = gp_models[i].kernel_function().noise();

                   return std::make_tuple(mu, sigma);
                }
                   // return _gp_model.query(x);
                return std::make_tuple(_gp_model.mu(x), Eigen::VectorXd::Zero(_gp_model.dim_out()));
            }

            void save_model(size_t iteration, std::string prefix = "") const
            {
                _gp_model.template save<limbo::serialize::TextArchive>(std::string(prefix + "model_learn_" + std::to_string(iteration)));
            }

            void load_model(const std::string& directory)
            {
                _gp_model.template load<limbo::serialize::TextArchive>(directory);
            }

        protected:
            GP_t _gp_model;
            bool _initialized = false;

            std::vector<Eigen::VectorXd> _to_vector(const Eigen::MatrixXd& m) const
            {
                std::vector<Eigen::VectorXd> result(m.rows());
                for (size_t i = 0; i < result.size(); ++i) {
                    result[i] = m.row(i);
                }
                return result;
            }
            std::vector<Eigen::VectorXd> _to_vector(Eigen::MatrixXd& m) const { return _to_vector(m); }

            Eigen::MatrixXd _to_matrix(const std::vector<Eigen::VectorXd>& xs) const
            {
                Eigen::MatrixXd result(xs.size(), xs[0].size());
                for (size_t i = 0; i < (size_t)result.rows(); ++i) {
                    result.row(i) = xs[i];
                }
                return result;
            }

            Eigen::MatrixXd _to_matrix(std::vector<Eigen::VectorXd>& xs) const { return _to_matrix(xs); }
        };
    } // namespace model
} // namespace skireil

#endif
