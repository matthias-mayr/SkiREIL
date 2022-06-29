# Making your own reward
## Introduction
This tutorial describes how to make your own reward.
At the end, there will be a new (and useless) reward function, but hopefully the ideas will be clear.

Reward functions are used in reinforcement learning to evaluate how well the currently learned behavior performed.
In SkiREIL, the reward of a single objective is a weighted sum of rewards. E.g., `total_reward = weight1*reward1 + weight2*reward2` where `weight1` and `weight2` are scalars, and `reward1` and `reward2` are functions that output a scalar at every time step.
`reward1` could be something like a mathematical description of closeness to the goal, and `reward2` could be distance from an obstacle to avoid.

## Configuration file
These rewards are defined in the json config file, e.g. in
`config/obstacle_task_multi_reward.json`

There is a block `"rewards":`, followed by a collection of rewards with unique names.  
Our first step is to add one here, that we still have to implement.
Let's say a thing we care about is the gripper not to be all open, because an open gripper makes the robot look scary.
Of course, to achieve e.g. a picking task, at some point, the gripper will need to open, in order to grasp the object.
That's ok, but we'll subtract some reward for that, so that the robot learns to only do it when really necessary.
Let's add `open_gripper_avoidance` as follows:
```json
"open_gripper_avoidance:": {
	"objective": "reward",
	"type": "OpenGripperAvoidanceReward",
	"negative": true,
	"weight": 30.0
}
```
Now, I've already made an important design choice: for `objective`, I've stated `reward`, like all other already defined rewards.
However, I could also turn this into a multi-objective optimization by stating another objective, such as `friendliness`.
Multi-objective optimization is a topic for another tutorial, so for now, we only have `reward` as the objective to maximize.
By convention, rewards are defined in a positive way, and we can flip the sign by stating `negative` to be `true`.
We're using `"OpenGripperAvoidanceReward"`, which does not exist yet. This is our next task.

## Implementing OpenGripperAvoidanceReward
In SkiREIL, reward functions are implemented in C++ in a cleverly named file found at `reward_functions.hpp`, which can be found at `include/skireil/reward/reward_functions.hpp`.

There, the reward structs consist of 3 sections: `configure`, `operator` and `private:`.
* `private:` includes the definitions of all private class variables used within the Reward struct.
* `configure` is a bunch of `if`-statements that parse the piece of json we defined earlier. Only if it can successfully parse every variable it expects, it will return `true`. In that case, (most of) the (private) class variables have obtained values and are ready to be used in `operator`. 
* `operator` is where the real magic happens: the current state of simulation is queried, and that information is plugged into a formula that defines the reward. Optionally, a feasibility check is performed. Finally, a pair `(reward, feasibility)` is returned.

You can copy an existing Reward struct and adapt those 3 sections. 
In `operator`, usually you would query the relevant part of the state of simulation, in our case something like `get_gripper_state(robot)` and use that in the reward function.
However, `get_gripper_state` doesn't exist, and it would lead us too far for this tutorial.
Defining a sensible operator() function *is left as an excercise to the reader*. Instead, we're just going to return a fixed reward of 0.

You might end up with something like this:
```cpp
template <typename RolloutInfo>
struct OpenGripperAvoidanceReward : public skireil::reward::Reward<RolloutInfo> {
    private:
        double _sign{1.0}; 

    bool configure(const nlohmann::json& conf, std::shared_ptr<robot_dart::Robot> robot, const std::vector<std::string>& robot_dof) {
        if (conf.contains("negative")) {
            if (conf["negative"].get<bool>()) {
                _sign = -1.0;
            }
        }
        return true;
    }

    std::pair<double, bool> operator()(RolloutInfo& info, const Eigen::VectorXd& from_state, const Eigen::VectorXd& action, const Eigen::VectorXd& to_state) const
    {
        double reward = 0.0;
		//reward = get_gripper_state(robot) == "open" ? -10 : 0;  // something like this
        reward = _sign * reward;
        return std::make_pair(reward, true);
    }
};
```

Note: `configure` will be run only once, parsing the json, while `operator` will be evaluated lots of times during simulation.
So, if you have any setup code that you need to run only once, e.g. to derive some information from inputs that remains constant during simulation, put it in `configure` and store the derived information in a private class variable.

## Registration
Before we can use our newly minted reward function, we have to 'register' it in a global map so that other parts of SkiREIL can access them easily.
For technical reasons, this map exists for every executable and the new definition needs to be added in:
* `src/dart/learning_skills.cpp`
* `src/dart/play_learning_skills.cpp`
 
There are lines that add entries to `reward_function_map` and the newly created reward function can be added as well:
```cpp
reward_function_map["OpenGripperAvoidanceReward"] = &createRewardFunction<skireil::reward::OpenGripperAvoidanceReward<rolloutinfo_t>>;
```

In order to use a new reward function, the code needs to be [recompiled](cloning_and_compiling.md).  
Have fun with your new reward function!

## Prototyping and Developing Rewards

It can often be helpful to get an understand of the values that a reward function calculates. It is possible to replay an earlier learned behavior while evaluating your new reward function.

In that case, your new reward should be included in the configuration json in the right `/tmp/skireil/` directory where the results of previous learning processes are saved. In case it is empty, it can either be populated with previous learning results that have been saved elsewhere or a short learning process can be started.

Every folder of an experiment in `/tmp/skireil/learning_skills/` has a file `scenario.json` that is a copy of the json file which was used for this experiment. The new reward function can be added there and a replay of some parameter files can be started. Replaying is explained [here](learning_and_replaying.md).  

