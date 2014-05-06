/*
Jarvis Schultz
May 2014

This node uses interactive markers to provide reference data for the receding
horizon optimal controller. If we are in global RUN state, this node sends a tf
and a point message at some frequency publishing the ineractive marker is. A
different node looks at this data and handles interpolation/time offseting for
the controller.

SUBSCRIPTIONS:
    - /operating_condition (OperatingCondition)

PUBLISHERS:
    - mass_ref_point (PointStamped)
    - mass_ref_frame (tf) ... not really a topic
    - visualization_markers (VisualizationMarkerArray)
*/


#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <puppeteer_msgs/OperatingCondition.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <vector>
#include <string>
#include <iostream>

///////////////////////////////
// GLOBAL CONSTANTS AND VARS //
///////////////////////////////
#define DT (0.01)
#define MARKERWF ("optimization_frame")
#define MARKERFRAME ("mass_ref_frame")
#define MARKERNAME ("ref_marker")

using namespace visualization_msgs;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
puppeteer_msgs::OperatingCondition op;
uint8_t operating_condition = op.IDLE;


// function for creating marker:
Marker makeMarker( InteractiveMarker &msg, std::string color)
{
    Marker marker;
    marker.type = Marker::SPHERE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.1;
    marker.color.g = 0.1;
    marker.color.b = 0.1;
    marker.color.a = 0.75;

    // mod color:
    if (color.compare("red") == 0)
	marker.color.r += 0.4;
    else if (color.compare("blue") == 0)
	marker.color.b += 0.4;
    else if (color.compare("green") == 0)
	marker.color.g += 0.4;
    else
	ROS_WARN("Marker color not recognized!");

    return marker;
}


InteractiveMarkerControl& makeMarkerControl( InteractiveMarker &msg, std::string color )
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back( makeMarker(msg, color) );
    msg.controls.push_back( control );
    
    return msg.controls.back();
}


void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    std::ostringstream s;
    if (operating_condition == op.RUN)
    {
	ROS_INFO("processFeedback called... type = %d", feedback->event_type);
	if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE)
	    ROS_INFO_STREAM(s.str() << "pose changed"
			    << "\nposition = "
			    << feedback->pose.position.x
			    << ", " << feedback->pose.position.y
			    << ", " << feedback->pose.position.z << std::endl);
			
	server->applyChanges();
    }
}

void SingleController(void)
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = MARKERWF;
    int_marker.pose.orientation.w = 1;
    int_marker.scale = 0.25;
    int_marker.name = MARKERNAME;
    int_marker.description = "set mass reference";

    makeMarkerControl(int_marker, "green");

    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
    return;
}


void opcb(const puppeteer_msgs::OperatingCondition &data)
{
    if (data.state < operating_condition)
    {
	ROS_INFO("Resetting interactive marker!");
	geometry_msgs::Pose tmp;
	tmp.orientation.w = 1;
	server->setPose(MARKERNAME, tmp);
    }
    operating_condition = data.state;
    if (operating_condition == op.RUN)
	SingleController();
    else if (operating_condition == op.IDLE)
	server->clear();
    server->applyChanges();
    return;
}


void send_transforms(void)
{
    static tf::TransformBroadcaster br;
    ros::Time tnow = ros::Time::now();
    InteractiveMarker vm;
    if (server->get(MARKERNAME, vm))
    {
	tf::Transform t;
	t.setOrigin(tf::Vector3(vm.pose.position.x, vm.pose.position.y, vm.pose.position.z));
	t.setRotation(tf::Quaternion(vm.pose.orientation.x, vm.pose.orientation.y,
				     vm.pose.orientation.z, vm.pose.orientation.w));
	br.sendTransform(tf::StampedTransform(t, tnow, MARKERWF, MARKERFRAME));
    }
    return;
}


void timercb(const ros::TimerEvent &e)
{
    if (operating_condition == op.RUN)
	send_transforms();
    return;
}




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "interactive_marker_ref_gen");
    ros::NodeHandle n;

    // create server for interactive markers:
    server.reset( new interactive_markers::InteractiveMarkerServer("mass_reference_control","",false) );

    ros::Duration(0.1).sleep();
    
    // create subscriber for operating condition
    ros::Subscriber opsub = n.subscribe("/operating_condition", 1, opcb);
    // publisher for point and visualization:
    ros::Publisher marker_pub = n.advertise<geometry_msgs::PointStamped>("mass_ref_point", 1);
    ros::Publisher con_pub = n.advertise<MarkerArray>("visualization_markers", 1);
    // timer for the publishing the data:
    ros::Timer pubtimer = n.createTimer(ros::Duration(DT), timercb);

    ros::spin();
    server.reset();
    
    return 0;
}


