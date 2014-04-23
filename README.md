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
Launch Controller.launch (uses accelerometers) or Vision_Tracking.launch (use camera). <br/> 
Algorithms can be tested with Webots (load the environment /first_approaches/nao.wbt). 

<UL TYPE="disc">
<LI> Nao Behavior Tree:
	<UL type="square">
	<LI> Tracking without obstacles: <br/>
	Launch CollaborativeMotion.launch (change robots IP in NaoBehaviorTree1/2).


	<LI> Tracking among obstacles: <br/>
	Create the obstacles map running ObstaclesCreation.launch (select the correct webcam). <br/>
	Launch ObstaclesNormal.launch.


	<LI> Example to execute a ROS Launch file: <br/>
	LD_LIBRARY_PATH=$PYTHONPATH:$LD_LIBRARY_PATH roslaunch nao_behavior_tree NaoBehaviorTree1.launch


	<LI> Color Filter calibration: <br/>
	Run ColorFilterCalibration.launch to choose the right HSV parameters. <br/>
	You can change then the values in parameters.yaml.
	</UL>
</UL>
