nao_collaborative_motion
========================

Collaborative motion planning of humanoid robots

Install Nao ROS package:
Follow instructions 3.2 v0.1 of http://wiki.ros.org/nao/Installation/remote.
Replace the script nao_camera.py.
Compile.

Add directories in .bashrc:
export AL_DIR=$HOME/Downloads/webots-for-nao/resources/projects/robots/nao/aldebaran/naoqi-runtime
export AL_DIR_SDK=$HOME/Downloads/webots-for-nao/resources/projects/robots/nao/aldebaran/simulator-sdk
export PYTHONPATH="$PYTHONPATH:$AL_DIR/lib"

First Approaches:
Launch Controller.launch or Vision_Tracking.launch.
/bin/bash: q: command not found

Example to launch a behavior tree:
LD_LIBRARY_PATH=$PYTHONPATH:$LD_LIBRARY_PATH roslaunch nao_behavior_tree NaoBehaviorTree1.launch
