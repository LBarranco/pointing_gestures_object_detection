#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "pointing_gestures_object_detection/Joints.h"
#include <iostream>

using namespace pointing_gestures_object_detection;

int main( int argc, char** argv )
{
	ros::init(argc, argv, "tracker_node");
	ros::NodeHandle node;
	
	Joints::Ptr outJoints(new Joints);
	
	// Frequency
	int frequency = 5;
	
	// Flag used to decide when the node can publish messages
	bool pub_flag = true;
	
	// Rate
	ros::Rate r(frequency);

	// Publisher declaration on the topic "right_joints" 
	ros::Publisher right_joints = node.advertise<Joints>("right_joints", 1);
	
	// Geometry points declaration for storing 3D coordinates of joints
	geometry_msgs::Point right_elbow_pose, right_hand_pose, p0;
	
	// Listener of the tf topic used to acquire the coordinates of joints
	tf::TransformListener listener;
	
	while (ros::ok())
	{ 
		// Previous point of the hand used to calculate the distance with the current hand point
		p0 = right_hand_pose;

		// Transforms declared for each joint
		tf::StampedTransform transform_right_elbow, transform_right_hand;	

		try
		{
			// Each joint frame to reference frame transforms. 
			// We use left frame (elbow and hand) because the Kinect detects users mirrored
			listener.lookupTransform("/camera_link", "/left_elbow_1",ros::Time(0), transform_right_elbow);
			listener.lookupTransform("/camera_link", "/left_hand_1",ros::Time(0), transform_right_hand);
		}
		catch (tf::TransformException &ex)
		{
			ROS_INFO("Please track yourself with the Kinect before beginning pointing\n");
			ros::Duration(2.0).sleep();
			continue;
		}

		// Joint position extraction and store:
		// right elbow joint
		right_elbow_pose.x = transform_right_elbow.getOrigin().x();
		right_elbow_pose.y = transform_right_elbow.getOrigin().y();
		right_elbow_pose.z = transform_right_elbow.getOrigin().z();
		// right hand joint
		right_hand_pose.x = transform_right_hand.getOrigin().x();
		right_hand_pose.y = transform_right_hand.getOrigin().y();
		right_hand_pose.z = transform_right_hand.getOrigin().z();
		
		// Used to avoid an invalid pointing position. Hand's joint must be closer to the Kinect than elbow's joint
		bool correct_pointing = right_elbow_pose.x > right_hand_pose.x;	

		// Filling joints message
        outJoints->r_elbow_pose = right_elbow_pose;
        outJoints->r_hand_pose = right_hand_pose;

		// Distance calculated between two subsequent hand's points
		double distance = sqrt( pow((double)p0.x - (double)right_hand_pose.x, 2)+
								pow((double)p0.y - (double)right_hand_pose.y, 2)+
								pow((double)p0.z - (double)right_hand_pose.z, 2));
		
		// Under this reference speed the pointing is considered admissible
		float v_ref = 0.025;
		
		// Distance theshold calculated in relation to the reference speed and the rate(frequency) of the node
		double d_treshold = v_ref * (1.0 /(double) frequency);
		
		// Setting pub_flag true when the user begin to move his right arm with a certain speed (3 times the distance trashold)
		if(!pub_flag && distance > 3*d_treshold){
			pub_flag = true;
		}
		
		// Under this threshold the arm is considered stationary and the message could be published
		if(distance < d_treshold && correct_pointing && pub_flag){							
			
			// Publishing of a Joints msg 
			right_joints.publish(outJoints);	
			pub_flag = false;
		}
		
   		r.sleep();
	}
}
