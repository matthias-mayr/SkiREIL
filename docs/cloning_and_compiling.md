# Cloning and Compiling this Repository
The install script to install SkiREIL is in `scripts/installation/install_skireil.sh`. It will install `ROS`, set up a catkin workspace and clone as well as compile the relevant repositories.

**Note:** In the first lines of that script the installation location is specified. By default it is `Workspaces/skireil_ws`.

Install with:
```bash
sudo apt-get install -y git lsb-release gnupg
git clone https://github.com/matthias-mayr/SkiREIL /tmp/skireil_installation
cd ~ # This is the folder into which it would be installed
/tmp/skireil_installation/scripts/installation/install_skireil.sh
rm -rf /tmp/skireil_installation # The clone in /tmp is only needed to execute the script
```
In order start experiments or recompile the `catkin` workspace, it is necessary to source the workspace in the `~/.bashrc` file. Add this line `source ~/Workspaces/skireil_ws/devel/setup.bash`. Adding can for example be done with:
```
echo "source ~/Workspaces/skireil_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/Workspaces/skireil_ws/src/skireil/scripts/paths.sh" >> ~/.bashrc
```

**Note:** `SkiREIL` sets up a soft link between `limbo` and the root of the `skireil` folder. ROS does not work well with that.
Therefore there is a script in `skireil/scripts/build/prepare_ros.sh` that needs to run before using ROS. The `skireil/scripts/build/configure.sh` script reverses those changes.

## Subsequent Compilations
After making changes to the project code, it needs to be compiled. This can be done like that:
```bash
roscd skireil
scripts/build/configure.sh
scripts/build/compile.sh
```
You can also add an alias to the `.bashrc` file to easen this process like this:
```
echo 'alias compile_skireil="roscd skireil && scripts/build/configure.sh && scripts/build/compile.sh"' >> ~/.bashrc
```