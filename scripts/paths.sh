# Get the directory the script is in
DIR=$(cd $( dirname ${BASH_SOURCE[0]} ) && pwd)

# get current directory
cwd=$DIR/..
cwd=$(realpath $cwd)  # get rid of '..'

# configure paths
# configure LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${cwd}/install/lib:${LD_LIBRARY_PATH}

# configure HYPERMAPPER_HOME
export HYPERMAPPER_HOME=${cwd}/deps/hypermapper
