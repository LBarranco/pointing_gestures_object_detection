#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "pointing_gestures_object_detection/Joints.h"
#include <iostream>
#include <visualization_msgs/Marker.h>

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
	
	// Publisher declaration on the topic "visualization_line" 
	ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("visualization_line", 1);
	
	// Set our initial shape type to be a line
	uint32_t shape = visualization_msgs::Marker::LINE_STRIP;
	
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
			listener.lookupTransform("/world", "/left_elbow_1",ros::Time(0), transform_right_elbow);
			listener.lookupTransform("/world", "/left_hand_1",ros::Time(0), transform_right_hand);
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
		
	

		// Variables used to contain the values of the x,y,z coordinates of r_elbow and r_hand
		float x1,y1,z1;
		float x2,y2,z2;

		x1 = right_hand_pose.x;
		y1 = right_hand_pose.y;
		z1 = right_hand_pose.z;

		x2 = right_elbow_pose.x;
		y2 = right_elbow_pose.y;
		z2 = right_elbow_pose.z;

		// Second point used to draw the straight line from the elbow
		geometry_msgs::Point endLinePoint;
		endLinePoint.x= 0.0;
		endLinePoint.y= y1 - ((x1 * (y2 - y1)) / (x2 - x1));
		endLinePoint.z= z1 - ((x1 * (z2 - z1)) / (x2 - x1));

		visualization_msgs::Marker marker;

		// Set the frame ID and timestamp.  See the TF tutorials for information on these.
		marker.header.frame_id = "/world";
		marker.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "line_strip";
		marker.id = 0;

		// Set the marker type
		marker.type = shape;

		// Set the marker action.
		marker.action = visualization_msgs::Marker::ADD;

		// Set the scale of the marker
		marker.scale.x = 0.01;

		// Set the two points for the line. It starts from elbow and finish to endLinePoint
		marker.points.push_back(right_elbow_pose);
		marker.points.push_back(endLinePoint);

		// Set the color -- be sure to set alpha(transparency) to something non-zero! (RGB 1.0/0.0/0.0 --> red)
		marker.color.r = 1.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		// Publish the marker
		marker_pub.publish(marker);

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
