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
#ifndef SKIREIL_REWARD_GP_REWARD_HPP
#define SKIREIL_REWARD_GP_REWARD_HPP

#include <vector>

#include <limbo/kernel/squared_exp_ard.hpp>
#include <limbo/mean/constant.hpp>
#include <limbo/model/gp.hpp>

#include <skireil/reward/reward.hpp>
#include <skireil/utils/utils.hpp>

namespace skireil {

    struct reward_defaults {
        struct kernel : public limbo::defaults::kernel {
            BO_PARAM(bool, optimize_noise, true);
        };

        struct kernel_squared_exp_ard : public limbo::defaults::kernel_squared_exp_ard {
        };

        struct mean_constant {
            BO_PARAM(double, constant, 0.);
        };

        struct opt_rprop : public limbo::defaults::opt_rprop {
            BO_PARAM(int, iterations, 300);
            BO_PARAM(double, eps_stop, 1e-4);
        };
    };

    template <typename Params>
    using RewardGP = limbo::model::GP<Params, limbo::kernel::SquaredExpARD<Params>, limbo::mean::Constant<Params>, skireil::model::gp::KernelLFOpt<Params, limbo::opt::Rprop<Params>>>;

    namespace reward {

        template <typename GP = RewardGP<reward_defaults>>
        struct GPReward : public Reward {
            template <typename RolloutInfo>
            double observe(RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state, bool keep = true)
            {
                double val = this(info, from_state, action, to_state);

                if (keep) {
                    Eigen::VectorXd sample = (this)->get_sample(info, from_state, action, to_state);
                    _samples.push_back(sample);
                    _obs.push_back(limbo::tools::make_vector(val));
                }

                return val;
            }

            template <typename RolloutInfo>
            double query(const RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state) const
            {
                Eigen::VectorXd mu;
                double sigma;
                std::tie(mu, sigma) = _model.query((this)->get_sample(info, from_state, action, to_state));

                return std::min(mu[0] + std::sqrt(sigma), std::max(mu[0] - std::sqrt(sigma), utils::gaussian_rand(mu[0], sigma)));
            }

            bool learn()
            {
                _model.compute(_samples, _obs, false);
                _model.optimize_hyperparams();

                return true;
            }

            template <typename RolloutInfo>
            Eigen::VectorXd get_sample(const RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state) const
            {
                Eigen::VectorXd vec(to_state.size() + action.size());
                vec.head(to_state.size()) = to_state;
                vec.tail(action.size()) = action;

                return vec;
            }

        protected:
            std::vector<Eigen::VectorXd> _samples, _obs;
            GP _model;
        };
    } // namespace reward
} // namespace skireil

#endif