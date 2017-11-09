#include <visualization_msgs/Marker.h>


void visualize_line(geometry_msgs::Point r_elbow_pose, geometry_msgs::Point r_hand_pose, ros::NodeHandle n);


void visualize_line(geometry_msgs::Point r_elbow_pose, geometry_msgs::Point r_hand_pose, ros::NodeHandle n)
{
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	// Set our initial shape type to be a line
  	uint32_t shape = visualization_msgs::Marker::LINE_STRIP;

	// Variables used to contain the velues of the x,y,z coordinates of r_elbow and r_hand
	float x1,y1,z1;
	float x2,y2,z2;
 		
	x1 = r_hand_pose.x;
	y1 = r_hand_pose.y;
	z1 = r_hand_pose.z;

	x2 = r_elbow_pose.x;
	y2 = r_elbow_pose.y;
	z2 = r_elbow_pose.z;
	
	// Second point used to draw the straight line from the elbow
	geometry_msgs::Point endLinePoint;
	endLinePoint.x= 0.0;
	endLinePoint.y= y1 - ((x1 * (y2 - y1)) / (x2 - x1));
	endLinePoint.z= z1 - ((x1 * (z2 - z1)) / (x2 - x1));
	
	visualization_msgs::Marker marker;

   	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
   	marker.header.frame_id = "/camera_link";
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
	marker.points.push_back(r_elbow_pose);
	marker.points.push_back(endLinePoint);
	
    // Set the color -- be sure to set alpha(transparency) to something non-zero! (RGB 1.0/1.0/1.0 --> white)
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
	
    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);
}


