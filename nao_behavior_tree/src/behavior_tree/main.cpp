#include <iostream>
#include <unistd.h>

#include "nao_behavior_tree/parameters.h"
#include "nao_behavior_tree/node.h"
#include "nao_behavior_tree/parser.h"
#include "nao_behavior_tree/display.h"
#include "nao_behavior_tree/keystroke.h"
#include "nao_behavior_tree/navigation.h"

NodeRoot root;					// the root of the bt
Node *node_cursor = NULL;		// used for displaying bt
Node *node = NULL;				// used for parsing bt
extern bool run;

int main(int argc, char** argv)
{
	// initialize the behavior tree client node
	ros::init(argc, argv, "nao_behavior_tree");

	ros::NodeHandle pnh("~");
	// File
	std::string file;
	pnh.param("file",file,std::string("/home/olivier/ros_workspace/nao_behavior_tree/bt.txt"));

	// initialize OpenGL engine for visualization
	glut_setup(argc, argv);

	// point to the root of the behavior tree
	node_cursor = node = &root;
	node_cursor->set_highlighted(true);

	// create the bt from the file bt.txt (put on the path)
	std::cout << "----------------- PARSE FILE -----------------" << std::endl;
	parse_file(file);

	// print the data parsed from the specification file
	std::cout << "----------------- PRINT TREE -----------------" << std::endl;
	root.print_subtree();

	// wait until user inputs a key + Enter to start (space + Enter does not work)
	int key;
	std::cout << "Press Key To Start" << std::endl;
	std::cin >> key;

	// start ticking the root of the tree at frequency: TICK_FREQUENCY
	while (ros::ok())
	{
		std::cout << "**** run" << run << std::endl;
		std::cout << "-------------- EXECUTE TREE --------------" << std::endl;
		root.execute_reset_status();
		root.execute();			// sending tick
		get_keypressed();		// processing keystrokes
		process_keypressed();
		glut_process();			// update visualization
		glutPostRedisplay();
		ros::Duration(1.0/TICK_FREQUENCY).sleep();
		std::cout << "**** run" << run << std::endl;
	}

	// missing to clear the dynamically allocated tree
	// delete_tree();

	return 0;
}
