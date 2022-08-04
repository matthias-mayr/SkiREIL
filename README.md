# SkiREIL Algorithm for Reinforcement Learning with Skill-based Systems using Behavior Trees

SkiREIL is approach to learn (single- and) multi-objective tasks by utilizing existing skills that expose learnable parameters. Targeted at industrial environments where tasks often have several objectives (KPIs) such as task performance and safety, it provides a way to retrieve a large variety of solutions from which the robot operator can choose from. By integrating planning and a knowledge framework it can offer a pipeline from high-level goal definition to the execution of learned policies on  a real system while conducting learning itself in highly parallelized simulations.

## What you should expect

This algorithm is a model-based policy search algorithm (the ICML 2015 [tutorial](http://icml.cc/2015/tutorials/PolicySearch.pdf) on policy search methods for robotics is a good source for reading) with the following main properties:

- it integrates with a policy representation that utilizes [SkiROS2](https://github.com/RVMI/skiros2) for skill representation and a motion generator implementation
- it provides a `json` format to quickly define and adapt learning scenarios --> see the `config` folder
- functionalities to implement reward functions, specific scenes and robot setups that can be specified in the scenario configuration file
- uses a simulator to model the dynamics of the robot/system
- is data-efficient or sample-efficient by utilizing Bayesian optimization, but can also use other optimization strategies
- parallelized evalution in simulation to profit from multi-core systems,
- it imposes no constraints on the type of the reward function (more specifically the reward function can also be learned from data)
- in principle it imposes no constraints on the type of the policy representation (any parameterized policy can be used --- for example, dynamic movement primitives or neural networks)

## Documentation

The documentation can be found in the [docs](docs) folder. The entry point [Readme has a table of contents](docs/README.md).


## Cloning and Compilation

More extensive documentation on how to clone, install and compile the code is in [docs/cloning_and_compiling.md](docs/cloning_and_compiling.md).

In short it would work like this:
```
git clone https://github.com/matthias-mayr/SkiREIL /tmp/skireil_installation
cd ~
/tmp/skireil_installation/scripts/installation/install_skireil.sh
```
## Start learning with SkiROS support
Open a terminal and enter:
```bash
tmux
scripts/experiments/launch_experiment_in_tmux.sh
# Specify in the lower right terminal which configuration file to use, so e.g.:
scripts/experiments/run_learning.sh 1 config/peg_insertion.json
```
If it is a multi-objective task, a postprocessing step needs to be run with:
```
roscd skireil && python3 scripts/experiments/create_param_config_and_pareto.py /tmp/skireil/learning_skills
```

More thorough documentation on how to start learning and what to expect is in [docs/learning_and_replaying.md](docs/learning_and_replaying.md).


## Code developers/maintainers

- Matthias Mayr (Lund University) --- actively developing/maintaining the code
- Konstantinos Chatzilygeroudis (Inria): http://costashatz.github.io/
- Rituraj Kaushik (Inria): http://tansigmoid.com/
- Roberto Rama (Inria)

Black-DROPS is orginally written under the ResiBots ERC Project (http://www.resibots.eu).


## Citing this Algorithm
Used in:
- IROS 2021 paper: Learning of Parameters in Behavior Trees for Movement Skills
- IROS 2022 submission: Skill-based Multi-objective Reinforcement Learning of Industrial Robot Tasks with Planning and Knowledge Integration

Based on code for the:
- IROS 2017 paper: "Black-Box Data-efficient Policy Search for Robotics"
- ICRA 2018 paper: "Using Parameterized Black-Box Priors to Scale Up Model-Based Policy Search for Robotics"

If you use our code for a scientific paper, please cite:

Mayr, M., Ahmad, F., Chatzilygeroudis, K., Nardi, L., Krueger, V. (2022) [Skill-based Multi-objective Reinforcement Learning of Industrial Robot Tasks with Planning and Knowledge Integration](https://arxiv.org/abs/2203.10033). *Submitted to IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*.

In BibTex:
  
```
@misc{https://doi.org/10.48550/arxiv.2203.10033,
  doi = {10.48550/ARXIV.2203.10033},
  url = {https://arxiv.org/abs/2203.10033},
  author = {Mayr, Matthias and Ahmad, Faseeh and Chatzilygeroudis, Konstantinos and Nardi, Luigi and Krueger, Volker},
  keywords = {Robotics (cs.RO), Machine Learning (cs.LG), FOS: Computer and information sciences, FOS: Computer and information sciences},
  title = {Skill-based Multi-objective Reinforcement Learning of Industrial Robot Tasks with Planning and Knowledge Integration},
  publisher = {arXiv},
  year = {2022},  
  copyright = {Creative Commons Attribution Share Alike 4.0 International}
    }
```

