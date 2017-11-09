#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include "pitt_msgs/TrackedShapes.h" 	// for out message (an array of TrackedShape), it internally include TrackedShape.h
#include "pointing_gestures_object_detection/Joints.h"

using namespace std;
using namespace pitt_msgs;

TrackedShapes::Ptr outShapes (new TrackedShapes);

//fills outShapes when the callback function is called
void trackedShapesCallback(const TrackedShapesConstPtr& msgTrackedShapes){
	
	//clearing OutShapes
	outShapes->tracked_shapes.clear();
	
	//temp vector: contains all the objects detected by the Pitt Package
	std::vector<TrackedShape> tmpVector(msgTrackedShapes->tracked_shapes);
	
	//filling outShapes with all the objects detected
	for(std::vector<TrackedShape>::iterator it = tmpVector.begin(); it != tmpVector.end(); ++it) {
		outShapes->tracked_shapes.push_back(*it);
	}
}

int main( int argc, char** argv )
{
	cout << "OBJECT ACQUISITION NODE " << endl << "Press enter to acquire the scene." << endl;
	ros::init(argc, argv, "object_acquisition_node");
	ros::NodeHandle node;
	
	// Rate
	ros::Rate r(5);

	// Subscriber of ransac_segmentation (Pitt Package)
	ros::Subscriber pitt_sub = node.subscribe("ransac_segmentation/trackedShapes", 1, trackedShapesCallback);
	
	// Publisher declaration on the topic "correct_shapes" 
	ros::Publisher shapes_pub = node.advertise<TrackedShapes>("correct_shapes", 1);
	
	while(ros::ok()){
		
		//when enter key is pressed
		if (cin.get() == '\n'){
			//calling callback
			ros::spinOnce();
			//publishing data
			shapes_pub.publish(outShapes);
			cout<<"published";
		}
		r.sleep();
	}
}
