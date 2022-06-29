# Add your own robot

## Introduction
**SkiREIL** is configured by default with a KUKA iiwa, but can in principle be configured with any robot.

At this time, a **Cartesian impedance controller** is the default controller being used and requires the robot has 7DoF.
It's explicitly verified that `Params::skireil::robot_dof` has 7 items.

In this tutorial, we are going to add a Franka Emika Panda.
It is the perfect candidate, because it has 7DoF and we know that the same Cartestian impedance controller is working on that robot in real life.

## Configuration file
In the .json config file, at root level, we can find `robot:`, with something that looks like this:
```
"robot": {
	"collisions": "mesh", 
	"tool": "peg_5mm", 
	"setup_name": "bh_rss"
},
```

Here, `collisions` is a parameter that can, for now, only be set to `mesh`.
The idea is that one could choose between an actual mesh or an approximation (e.g. a convex hull).
`mesh` will use an actual robot mesh to compute collisions, which is more costly to compute, but also more accurate than approximations.

`tool` is a parameter that specifies the tool attached to the end effector.
In the paper, it's already being swapped out between different sizes of pegs.

Lastly, `setup_name` refers to the actual robot.

## Defining a Robot
Let's visit `include/skireil/experiments/learning_skills_robots.hpp`.  
At the top, we can see the namespace is `learning_skills::robots`. Besides a `load_robot` method, it's full of robot definitions.
Let's add ours:

```cpp
template <typename Params>
std::shared_ptr<robot_dart::Robot> panda(const json& j) {
	// We need packages to desribe where the stl files are located.
	// They are copied into SkiREIL to avoid dependencies.
	std::vector<std::pair<std::string, std::string>> packages;
	packages.push_back(std::make_pair(std::string("panda_description"), std::string(RESPATH)));
	std::string robot_file {std::string(RESPATH) + "/URDF/panda.urdf"};
	Params::skireil::set_robot_end_effector("panda_hand");
	std::shared_ptr<robot_dart::Robot> robot = load_robot(robot_file, packages, "arm");
	Params::skireil::set_robot_dof(robot->dof_names(true, true, true));
	if (Params::skireil::robot_dof().size() != 7) {
		throw std::runtime_error("Expected 7 DOF, got " + std::to_string(Params::skireil::robot_dof().size()));
	}
	robot->set_actuator_types(Params::dart_policy_control::joint_type());

	Params::skireil::set_action_dim(19);
	Params::skireil::set_command_dim(7);
	Params::skireil::set_model_input_dim(14);
	Params::skireil::set_model_pred_dim(14);
	return robot;
```

This is where we refer to the URDF files that describe the robot.

Note: `RESPATH` refers to the `res` folder in this project. It is where the URDF files are located. The `packages.push_back(...);` calls make sure that the URDFs can access the meshes in those packages. If you've referred to an external robot description package, make sure to include it here.

## URDF
Usually, robots are described using [Unified Robot Description Format](https://mathworks.com/help/physmod/sm/ug/urdf-model-import.html), an XML description of your robot.
Information like how the robot's joints relate to each other are contained in this file.
Chances are some friendly people have already published a URDF for your robot.
A quick search reveals [a repository with a URDF description of the panda](https://github.com/ros-planning/moveit_resources/tree/master/panda_description).
We'll place the URDF file under `res/URDF` and the .stl meshes under `res/meshes/panda`.
These will come in handy later on.

## Modifying the URDF
### Attaching other stuff
We need the panda to be attached to the world, otherwise it will gain 6 degrees of freedom. An additional link can be configured by adding this:
```
    <link name="world"/>
    <joint name="panda_base_joint" type="fixed">
        <parent link="world"/>
        <child link="panda_link0"/>
        <origin rpy="0 0 0" xyz="-0.1 0.4 0.85"/>
    </joint>
```
It makes sense to add this in the very beginning of the file, since `panda_link0` is a child of `world`. Usually, the further down in an URDF, the closer to end effector. The translation (`xyz="-0.1 0.4 0.85"`) allows us to position the Panda.

At the very end of the file, we might want to add the tool we're planning on using, say the `peg`:
```
  <joint name="peg_end_joint" type="fixed">
    <parent link="panda_rightfinger"/>
    <child link="peg"/>
    <origin rpy="0 0 0" xyz="0 0 0.1"/>
  </joint>
  <link name="peg"/>
```
In this case we didn't spend time thinking about where the peg should be exactly, and just attached it to the `panda_rightfinger`.

### Referring to the right Package
In the URDF file, you'll see some `<mesh>` tags that have a `filename=` property that starts with `package://`.

The easiest way to allow to resolve them is to add additional lookups in the `packages` vector. For example to be able to find
```xml
<mesh filename="package://panda_description/meshes/panda/visual/link_0.stl"/>
```
the file needs to be located in `res/meshes/panda/visual/link_0.stl` and it can be made discoverable by mapping `panda_description` to the `res` folder like this:
```cpp
packages.push_back(std::make_pair(std::string("panda_description"), std::string(RESPATH)));
```

## Registering a robot in the robot map
As a final step, the new robot configuration needs to be registered just like with scenes and rewards. It allows to map setup names in the json file to robot configuration functions.

The mapping is located in the file `include/skireil/experiments/learning_skills.hpp` inside of the `get_robot_map()` method.
Let's add `panda` there:
```cpp
robot_map["panda"] = &robots::panda<Params>;
```

Now let's change `setup_name` to the new `panda` definition in the json file:
```
"robot": {
	"collisions": "mesh", 
	"tool": "peg_5mm", 
	"setup_name": "panda"
},
```

After a [recompilation](cloning_and_compiling.md) the new setup is available and learning with it can be started.

## Additional Attributes
The json definition of the `robot` block is free to choose and append. The only required attribute is `setup_name`. It is possible to define custom attributes like `tool`, `collisions`, `color`, etc and interprete them in the robot loading function.

## Troubleshooting
* The package name has to be right in `learning_skills_robots.hpp`:   
  `moveit_resources_panda_description` is not `panda_description`, although the folder name is just `panda_description`. The package name can be found in `package.xml`.
* `degrees of freedom do not match 7`: caused by `world` not being present in the URDF. If you still have too many degrees of freedom, try setting some (e.g. fingers) to `fixed`.
* `robot_dart assertion failed: dof_data: panda_joint1 is not in dof_map`: caused by a URDF that didn't include `world` properly as a link.

