nao_collaborative_motion
========================

Collaborative motion planning of humanoid robots

- Install Nao ROS package: <br/> 
Follow instructions 3.2 v0.1 of http://wiki.ros.org/nao/Installation/remote. <br/>
Replace the script nao_camera.py. <br/>
Compile.

- Add directories in .bashrc: <br/>
export AL_DIR=$HOME/Downloads/webots-for-nao/resources/projects/robots/nao/aldebaran/naoqi-runtime <br/>
export AL_DIR_SDK=$HOME/Downloads/webots-for-nao/resources/projects/robots/nao/aldebaran/simulator-sdk <br/>
export PYTHONPATH="$PYTHONPATH:$AL_DIR/lib"

- First Approaches: <br/>
Launch Controller.launch or Vision_Tracking.launch. <br/> 
Algorithms can be tested with Webots (load the environment /first_approaches/nao.wbt). 

- Example to execute a ROS Launch file: <br/>
LD_LIBRARY_PATH=$PYTHONPATH:$LD_LIBRARY_PATH roslaunch nao_behavior_tree NaoBehaviorTree1.launch 
