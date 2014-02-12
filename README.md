nao_collaborative_motion
========================

Collaborative motion planning of humanoid robots

Bashrc:
export AL_DIR=$HOME/Downloads/webots-for-nao/resources/projects/robots/nao/aldebaran/naoqi-runtime
export AL_DIR_SDK=$HOME/Downloads/webots-for-nao/resources/projects/robots/nao/aldebaran/simulator-sdk
export PYTHONPATH="$PYTHONPATH:$AL_DIR/lib"

Launch BT:
LD_LIBRARY_PATH=$PYTHONPATH:$LD_LIBRARY_PATH roslaunch nao_behavior_tree NaoBehaviorTree1.launch
