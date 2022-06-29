
## Start learning with SkiROS support

In order to start learning with SkiROS support, these components need to be started:
1. roscore
2. robot bringup to make the robot description available for the world model
3. SkiROS
4. Learning algorithm

### Automated Startup in tmux
The above-mentioned steps are automated in a script in `scripts/experiments/launch_experiment_in_tmux.sh`. It assumes that a tmux session is already start. You can do so like this:
```bash
tmux
scripts/experiments/launch_experiment_in_tmux.sh
```
After the startup process is completed, the lower right terminal allows to specify which experiment to run with

```
scripts/experiments/run_learning.sh <#REPETITIONS> config/CONFIG_FILE.json
```

### Manual Startup
**Note:** The `prepare_ros.sh` scripts needs to run before one can launch things from `skireil`

These are necessary for the underlying ROS nodes and the policy:
```bash
roscore
# New terminal
roscd skireil
scripts/build/prepare_ros.sh
mon launch skireil robot_bringup.launch learning:=true sim:=true
# New terminal
mon launch skireil skiros.launch learning:=true
```

After that, skireil can be start in a new terminal with
```
scripts/build/configure.sh
scripts/experiments/run_learning.sh <#REPETITIONS> config/CONFIG_FILE.json
```

### Postprocessing
Multi-objective tasks need a postprocessing step to find the pareto-optimal points and create parameter configurations for them.

- If the experiment folder is not in `/tmp/skireil/learning_skills`, copy it there
- Execute `roscd skireil && python3 scripts/experiments/create_param_config_and_pareto.py /tmp/skireil/learning_skills/<experiment_folder>/`

It is also possible to do the postprocessing step for all folders in `/tmp/skireil/learning_skills`. This can be done with:  
`roscd skireil && python3 scripts/experiments/create_param_config_and_pareto.py /tmp/skireil/learning_skills`.

#### Plotting Pareto Frontiers
The postprocessing step described above will also create Pareto frontiers with a script that is part of `hypermapper`.

A manual script for nicer looking frontiers for multiple experiments is in `scripts/plotting/paretos.py`. This step requires that postprocessing for every experiment has been done first. 


## Playing Results of the Learning Process

Often one wants to play parameter configurations of learned behaviors in simulation. E.g. to evaluate the outcome of a learning process. This can be done with:
```bash
roscore
# New terminal
scripts/build/prepare_ros.sh
mon launch skireil robot_bringup.launch learning:=true sim:=true
# New terminal
mon launch skireil skiros.launch learning:=true
# New terminal
scripts/experiments/play_param_config.sh
```

This executables requires the experiments to be located in `/tmp/skireil/learning_skills/`. An interactive dialogue allows to choose an experiment and a parameter configuration.

