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
    - limit_markers (Marker)
*/


#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <puppeteer_msgs/OperatingCondition.h>
#include <puppeteer_msgs/PlanarSystemConfig.h>
#include <puppeteer_msgs/PlanarSystemService.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <XmlRpcValue.h>

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
ros::Publisher *marker_pub_ref;
ros::Publisher *con_pub_ref;
ros::Publisher *limits_pub_ref;
geometry_msgs::Pose start_pose;
puppeteer_msgs::OperatingCondition op;
uint8_t operating_condition = op.IDLE;
double xmax,xmin,ymax,ymin;
bool limits_bool=false;
Marker limit_marker;


// function for creating marker of limits:
Marker makeLimitMarker(void)
{
    Marker marker;
    marker.type = Marker::LINE_STRIP;
    marker.scale.x = 0.05;
    marker.color.a = 1.0f;
    marker.points.resize(5);
    marker.lifetime = ros::Duration();
    marker.header.frame_id = MARKERWF;

    // Lower left
    marker.points[0].x = xmin;
    marker.points[0].y = ymin;
    // Upper left
    marker.points[1].x = xmin;
    marker.points[1].y = ymax;
    // Upper right
    marker.points[2].x = xmax;
    marker.points[2].y = ymax;
    // Lower right
    marker.points[3].x = xmax;
    marker.points[3].y = ymin;
    // repeat lower left
    marker.points[4].x = xmin;
    marker.points[4].y = ymin;

    return marker;
}

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
    if (operating_condition == op.RUN)
    {
	// std::ostringstream s;
	// ROS_INFO("processFeedback called... type = %d", feedback->event_type);
	// if (feedback->event_type == InteractiveMarkerFeedback::POSE_UPDATE)
	//     ROS_INFO_STREAM(s.str() << "pose changed"
	// 		    << "\nposition = "
	// 		    << feedback->pose.position.x
	// 		    << ", " << feedback->pose.position.y
	// 		    << ", " << feedback->pose.position.z << std::endl);
	if (limits_bool)
	{
	    geometry_msgs::Pose pose = feedback->pose;
	    // check min/max vals and project:
	    if (pose.position.x > xmax)
	    	pose.position.x = xmax;
	    if (pose.position.x < xmin)
	    	pose.position.x = xmin;
	    if (pose.position.y > ymax)
	    	pose.position.y = ymax;
	    if (pose.position.y < ymin)
	    	pose.position.y = ymin;
	    server->setPose(feedback->marker_name, pose);
	}
	server->applyChanges();
    }
}


void SingleController(void)
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = MARKERWF;
    int_marker.pose.orientation.w = 1.0;
    int_marker.pose.position.x = start_pose.position.x;
    int_marker.pose.position.y = start_pose.position.y;
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
    // if (data.state < operating_condition)
    // {
    // 	ROS_INFO("Resetting interactive marker!");
    // 	server->setPose(MARKERNAME, start_pose);
    // }
    if (operating_condition == data.state)
	return;
    operating_condition = data.state;
    if (operating_condition == op.RUN)
    {
	SingleController();
	limits_pub_ref->publish(limit_marker);
	// server->applyChanges();
	// server->setPose(MARKERNAME, start_pose);
    }
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
    std::vector<visualization_msgs::Marker> mlist;
    if (server->get(MARKERNAME, vm))
    {
	tf::Transform t;
	t.setOrigin(tf::Vector3(vm.pose.position.x, vm.pose.position.y, vm.pose.position.z));
	t.setRotation(tf::Quaternion(vm.pose.orientation.x, vm.pose.orientation.y,
				     vm.pose.orientation.z, vm.pose.orientation.w));
	br.sendTransform(tf::StampedTransform(t, tnow, MARKERWF, MARKERFRAME));
	geometry_msgs::PointStamped pt;
	pt.header.stamp = tnow;
	pt.header.frame_id = MARKERWF;
	pt.point.x = vm.pose.position.x;
	pt.point.y = vm.pose.position.y;
	pt.point.z = vm.pose.position.z;
	marker_pub_ref->publish(pt);
	// build entry for MarkerArray
	Marker m;
	m.header = vm.header;
	m.pose = vm.pose;
	m.type = vm.controls[0].markers[0].type;
	m.color = vm.controls[0].markers[0].color;
	m.scale = vm.controls[0].markers[0].scale;
	mlist.push_back(m);
    }
    MarkerArray ma;
    ma.markers = mlist;
    con_pub_ref->publish(ma);
    return;
}


void timercb(const ros::TimerEvent &e)
{
    if (operating_condition == op.RUN)
	send_transforms();
    return;
}


void get_initial_config(geometry_msgs::Pose &start_pose)
{
    // wait for service, and get the starting config:
    ROS_INFO("interactive marker node Waiting for get_ref_config service:");
    ROS_INFO("Will not continue until available...");
    if(ros::service::waitForService("get_ref_config"))
    {
	// service available:
	puppeteer_msgs::PlanarSystemService::Request req;
	puppeteer_msgs::PlanarSystemService::Response resp;
	puppeteer_msgs::PlanarSystemConfig q0;
	req.index = 0; //initial config:
	ros::service::call("get_ref_config", req, resp);
	q0 = resp.config;
	ROS_DEBUG("Initial config:");
	ROS_DEBUG("xm=%f, ym=%f, xr=%f, r=%f\r\n",q0.xm,q0.ym,q0.xr,q0.r);
	start_pose.position.x = q0.xm;
	start_pose.position.y = q0.ym;
	start_pose.position.z = 0.0;
	start_pose.orientation.w = 1;
	start_pose.orientation.x = 0;
	start_pose.orientation.y = 0;
	start_pose.orientation.z = 0;
    }
    else
    {
	ROS_ERROR("Could not get service, going to assume initial pose is zero");
	start_pose.position.x = 0;
	start_pose.position.y = 0;
	start_pose.position.z = 0;
	start_pose.orientation.w = 1;
	start_pose.orientation.x = 0;
	start_pose.orientation.y = 0;
	start_pose.orientation.z = 0;
    }
    return;
}


void setup_limits(void)
{
    if (ros::param::has("xlim") && ros::param::has("ylim"))
    {
	// then we will enforce limits:
	limits_bool = true;
	// get the values:
	XmlRpc::XmlRpcValue param_list;
	ros::param::get("xlim", param_list);
	ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(param_list.size() == 2);
	xmin = static_cast<double>(param_list[0]);
	xmax = static_cast<double>(param_list[1]);
	ROS_INFO("Limits read in for interactive marker node:");
	ROS_INFO("xlim = [%f, %f]",xmin, xmax);

	ros::param::get("ylim", param_list);
	ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
	ROS_ASSERT(param_list.size() == 2);
	ymin = static_cast<double>(param_list[0]);
	ymax = static_cast<double>(param_list[1]);
	ROS_INFO("ylim = [%f, %f]",ymin, ymax);

	// build marker if we need to:
	limit_marker = makeLimitMarker();
    }
    else
	ROS_WARN("Not enforcing interactive limits!");
}




int main(int argc, char *argv[])
{
    ros::init(argc, argv, "interactive_marker_ref_gen");
    ros::NodeHandle n;

    // fill in initial config:
    get_initial_config(start_pose);
    ROS_INFO("Marker start pose:");
    ROS_INFO("x = %f y = %f",start_pose.position.x, start_pose.position.y);

    // setup limits:
    setup_limits();

    // create server for interactive markers:
    server.reset( new interactive_markers::InteractiveMarkerServer("mass_reference_control","",false) );

    ros::Duration(0.1).sleep();

    // create subscriber for operating condition
    ros::Subscriber opsub = n.subscribe("/operating_condition", 1, opcb);
    // publisher for point and visualization:
    ros::Publisher marker_pub = n.advertise<geometry_msgs::PointStamped>("mass_ref_point", 1);
    marker_pub_ref = &marker_pub;
    ros::Publisher con_pub = n.advertise<MarkerArray>("visualization_markers", 1);
    con_pub_ref = &con_pub;
    ros::Publisher limits_pub = n.advertise<Marker>("limit_markers", 1);
    limits_pub_ref = &limits_pub;

    if (limits_bool)
	limits_pub_ref->publish(limit_marker);

    // timer for the publishing the data:
    ros::Timer pubtimer = n.createTimer(ros::Duration(DT), timercb);

    ros::spin();
    server.reset();

    return 0;
}


