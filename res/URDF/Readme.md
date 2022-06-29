# Generate iiwa urdf

Use the command
```
rosrun xacro xacro iiwa_left.urdf.xacro publish_robot:=true > iiwa_left.urdf
```
to generate the URDF from the xacro. Of course this command needs a working ROS configuration as well as access to all our iiwa description files in `bh_description`.

Manual changes needed to be applied to the the collisions of the iiwa model since DART only seems to be able to handle boxes and ellipsoids. So if the command is invoked again they would be overwritten.

Values are taken from here: https://github.com/mosra/magnum-examples/blob/08cddf48fdfcf5ffe2be51a211c387ea2cc8fa9e/src/dart/urdf/iiwa14_simple.urdf
Following this tutorial: https://blog.magnum.graphics/guest-posts/using-dart-to-control-a-robotic-manipulator/

## Generate gripper URDF

```
rosrun xacro xacro robotiq_arg2f_85_model.xacro > robotiq_2f.urdf 
```