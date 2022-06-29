#!/bin/bash
set -e

print_help() {
    echo "Entry point in the container. No option starts a shell"
    echo -e "\nOptions:"
    echo -e "\t -e|--experiment EXPERIMENT\tStarts an experiment with the given name"
    echo -e "\t -h|--help\tPrint help"
    exit 0
}

EXP=""

while [[ $# -gt 0 ]]; do
  case $1 in
    -e|--experiment)
      EXP=$2
      shift # past argument
      shift # past argument
      ;;
    -h|--help)
      print_help
      exit 0
      ;;
    -*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
    *)
      POSITIONAL_ARGS+=("$1") # save positional arg
      shift # past argument
      ;;
  esac
done
set -- "${POSITIONAL_ARGS[@]}" # restore positional parameters

. /Workspaces/skireil_ws/devel/setup.bash
cd /Workspaces/skireil_ws/src/skireil
. scripts/paths.sh
roscd skireil

if [ ! -z "${EXP}" ] ; then
  echo "Experiment to run:" $EXP
  scripts/experiments/launch_experiment_in_tmux.sh "$@"
else
  echo "Starting shell"
fi
exec $SHELL