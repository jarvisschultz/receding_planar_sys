// planar_coordinator.cpp
// Jarvis Schultz
// Fall 2012

//---------------------------------------------------------------------------
// Notes
// ---------------------------------------------------------------------------
// This node is used for coordinating the output of two of the object
// tracking nodes. One for tracking the robot, and one for tracking
// the suspended mass.  This node subscribes to the outputs of each
// node, and publishes the results of both incoming PointPlus messages
// as a puppeteer_msgs/PlanarSystemConfig message.  It is also
// responsible for handling all of the logic related to the
// /operating_condition and performing the calibration.


///////////////////////////////////////////////////////////////////////////
// // SUBSCRIPTIONS:							 //
// //	- robot_kinect_position (puppeteer_msgs::PointPlus)		 //
// //	- object1_position (puppeteer_msgs::PointPlus)			 //
// //									 //
// // PUBLISHERS:							 //
// //	- meas_config (puppeteer_msgs::PlanarSystemConfig)		 //
// //									 //
// // SERVICE CALLS:							 //
// //	- get_ref_config (puppeteer_msgs::PlanarSystemService)		 //
///////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <iostream>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <puppeteer_msgs/PointPlus.h>
#include <puppeteer_msgs/PlanarSystemConfig.h>
#include <puppeteer_msgs/PlanarSystemService.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
// local:
#include "state_intp.hpp"


//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define NUM_CALIBRATES (30)
#define NUM_EKF_INITS (3)
#define ROBOT_CIRCUMFERENCE (57.5) // centimeters
#define DEFAULT_ROBOT_RADIUS (ROBOT_CIRCUMFERENCE/M_PI/2.0/100.) // meters
#define DEFAULT_MASS_RADIUS (0.05285/2.0) // meters
#define H0 (1.0) // default height of robot in meters
#define NUMBER_CONFIGS (20) // max number of measured configs to store at a time

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------
using namespace message_filters;
using namespace puppeteer_msgs;


class PlanarCoordinator {

private:
    ros::NodeHandle node_;
    ros::Publisher config_pub;
    message_filters::Subscriber<PointPlus> robot_kinect_sub;
    message_filters::Subscriber<PointPlus> mass_kinect_sub;
    message_filters::TimeSynchronizer<PointPlus, PointPlus> * sync;
    double robot_radius;
    PlanarSystemConfig q0;
    bool calibrated_flag;
    unsigned int calibrate_count;
    Eigen::Vector3d robot_cal_pos, mass_cal_pos, cal_pos;
    Eigen::Vector3d robot_start_pos, mass_start_pos;
    tf::TransformListener tf;
    tf::TransformBroadcaster br;
    std::vector<double> *tvec;
    std::vector<state_type> *qvec;
    
    
public:
    PlanarCoordinator () :
	tvec = new std::vector<double>(),
	qvec = std::vector<
	{
	ROS_DEBUG("Instantiating a PlanarCoordinator Class");
	// define subscribers, synchronizer, and the corresponding
	// callback:
	robot_kinect_sub.subscribe(node_, "robot_kinect_position", 10);
	mass_kinect_sub.subscribe(node_, "object1_position", 10);
	sync = new message_filters::TimeSynchronizer<PointPlus, PointPlus>
	    (robot_kinect_sub, mass_kinect_sub, 10);
	sync->registerCallback(boost::bind(
				   &PlanarCoordinator::callback, this, _1, _2));
	// define publisher
	config_pub = node_.advertise<PlanarSystemConfig> ("meas_config", 100);

	// wait for service, and get the starting config:
	ROS_INFO("Waiting for get_ref_config service:");
	ROS_INFO("Will not continue until available...");
	if(ros::service::waitForService("get_ref_config"))
	{
	    // service available:
	    PlanarSystemService::Request req;
	    PlanarSystemService::Response resp;
	    req.index = 0; //initial config:
	    ros::service::call("get_ref_config", req, resp);
	    q0 = resp.config;
	    ROS_DEBUG("Initial config:");
	    ROS_DEBUG("xm=%f, ym=%f, xr=%f, r=%f\r\n",q0.xm,q0.ym,q0.xr,q0.r);
	}
	else
	{
	    ROS_ERROR("Could not get service, shutting down");
	    ros::shutdown();
	}

	// set all default variables:
	calibrated_flag = false;
	calibrate_count = 0;
	
	
	return;
    }

    ~PlanarCoordinator() {
	delete sync;
	return;
    }

    void callback(const PointPlusConstPtr& robot_point, const PointPlusConstPtr& mass_point)
	{
	    ROS_DEBUG("Synchronized Callback triggered");
	    puppeteer_msgs::PointPlus r_pt, m_pt;

	    // if not in run or calibrate, just exit
	    if (run_system_logic())
		return;

	    // let's first correct for the radii of the objects:
	    r_pt = *robot_point;
	    r_pt = correct_vals(r_pt, DEFAULT_ROBOT_RADIUS);
	    m_pt = *mass_point;
	    m_pt = correct_vals(m_pt, DEFAULT_MASS_RADIUS);

	    if (calibrated_flag == false)
	    {
		if (calibrate_count == 0)
		{
		    calibrate_count++;
		    robot_cal_pos << 0, 0, 0;
		    mass_cal_pos << 0, 0, 0;
		    robot_start_pos << 0, 0, 0;
		    mass_start_pos << 0, 0, 0;
		    return;
		}
		else if (calibrate_count <= NUM_CALIBRATES)
		{
		    ROS_INFO_THROTTLE(1, "Calibrating...");
		    if (!m_pt.error && !r_pt.error)
		    {
			// then we got both objects successfully:
			robot_cal_pos(0) += r_pt.x;
			robot_cal_pos(1) += r_pt.y;
			robot_cal_pos(2) += r_pt.z;
			mass_cal_pos(0) += m_pt.x;
			mass_cal_pos(1) += m_pt.y;
			mass_cal_pos(2) += m_pt.z;
			calibrate_count++;
		    }
		    return;
		}
		else
		{
		    ROS_INFO("Calibration completed successfully!");
		    // if here, calibration is done!
		    // first find averages:
		    robot_cal_pos = (robot_cal_pos/((double) NUM_CALIBRATES));
		    mass_cal_pos = (mass_cal_pos/((double) NUM_CALIBRATES));
		    // now set start positions:
		    mass_start_pos << q0.xm, q0.ym, 0;
		    robot_start_pos << q0.xr, H0, 0;
		    // now get the transform:
		    mass_cal_pos -= mass_start_pos;
		    robot_cal_pos -= robot_start_pos;
		    // let's check if there is an error here:
		    ////////////////////////
                    // ERROR CHECK	  //
                    ////////////////////////
		    // if no error:
		    // cal_pos = (mass_cal_pos + robot_cal_pos)/2.0;
		    cal_pos = mass_cal_pos;

		    // reset vars:
		    calibrate_count = 0;
		    calibrated_flag = true;
		}
	    }
	    
	    // send global frame transforms:
	    send_global_transforms(m_pt.header.stamp);

	    // transform both of the points into the
	    // optimization_frame, build the output message, and
	    // publish
	    tf::Transform transform;
	    PlanarSystemConfig qmeas;
	    Eigen::Affine3d gwo;
	    Eigen::Vector3d robot_trans, mass_trans;
	    transform.setOrigin(tf::Vector3(cal_pos(0),
					    cal_pos(1), cal_pos(2)));
	    transform.setRotation(tf::Quaternion(0,0,0,1));
	    tf::TransformTFToEigen(transform, gwo);
	    gwo = gwo.inverse();
	    // transform robot:
	    robot_trans << r_pt.x, r_pt.y, r_pt.z;
	    robot_trans = gwo*robot_trans;
	    // transform mass:
	    mass_trans << m_pt.x, m_pt.y, m_pt.z;
	    mass_trans = gwo*mass_trans;
	    // calculate string length:
	    mass_trans(2) = 0; robot_trans(2) = 0;
	    double rad = (mass_trans - robot_trans).norm();
	    qmeas.xm = mass_trans(0);
	    qmeas.ym = mass_trans(1);
	    qmeas.xr = robot_trans(0);
	    qmeas.r = rad;
	    qmeas.mass_err = m_pt.error;
	    qmeas.robot_err = r_pt.error;
	    qmeas.header.stamp = m_pt.header.stamp;
	    qmeas.header.frame_id = "optimization_frame";
	    config_pub.publish(qmeas);
		    
	    return;
	}

    // run all logic related to the operating condition... return
    // 'true' if we should exit the callback
bool run_system_logic(void)
	{
	    int operating_condition = 0;
	    static int last_op_con = operating_condition;
	    if(ros::param::has("/operating_condition"))
	    {
		ros::param::getCached("/operating_condition", operating_condition);
	    }
	    else
	    {
		ROS_WARN("Cannot Find Parameter: operating_condition");
		ROS_INFO("Setting operating_condition to IDLE");
		ros::param::set("/operating_condition", 0);
	    }
	    
	    bool quit_cb_flag = false;
	    if (operating_condition < last_op_con)
	    {
		ROS_DEBUG("Need to calibrate due to state downgrade");
		calibrated_flag = false;
		calibrate_count = 0;
	    }
	    if (calibrated_flag) // if we're calibrated, callback should always run
	    {
		quit_cb_flag = false;
	    }
	    else if (operating_condition != op_con_msg.CALIBRATE)
	    {
		quit_cb_flag = true;
	    }
	    else
	    {
		quit_cb_flag = false;
	    }
	    last_op_con = operating_condition;
	    return quit_cb_flag;
	}

    
    // this function accounts for the size of the robot:
    puppeteer_msgs::PointPlus correct_vals(puppeteer_msgs::PointPlus &p, double radius)
	{
	    ROS_DEBUG("correct_vals called");
	    puppeteer_msgs::PointPlus point;
	    point = p;
	    	    
	    // let's create a unit vector from the kinect frame to the
	    // robot's location
	    Eigen::Vector3d ur;
	    ur << p.x, p.y, p.z;
	    // now turn it into a unit vector:
	    ur = ur/ur.norm();
	    // now we can correct the values of point
	    ur = ur*radius;
	    
	    point.x = point.x+ur(0);
	    point.y = point.y+ur(1);
	    point.z = point.z+ur(2);
	    
	    return(point);	    	    
	}


    // this is a function for sending all of the global transforms
    // that we want to send:
    void send_global_transforms(ros::Time tstamp)
	{
	    tf::Transform transform;
	    // first publish the optimization frame:
	    transform.setOrigin(tf::Vector3(cal_pos(0),
					    cal_pos(1),
					    cal_pos(2)));
	    transform.setRotation(tf::Quaternion(0,0,0,1));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "oriented_optimization_frame",
						  "optimization_frame"));

	    
	    // publish the map frame at same height as the Kinect
	    transform.setOrigin(tf::Vector3(cal_pos(0),
					    0,
					    cal_pos(2)));
	    transform.setRotation(tf::Quaternion(.707107,0.0,0.0,-0.707107));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "oriented_optimization_frame",
						  "map"));
	    
	    // publish one more frame that is the frame the robot
	    // calculates its odometry in.
	    transform.setOrigin(tf::Vector3(0,0,0));
	    transform.setRotation(tf::Quaternion(1,0,0,0));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "map",
						  "robot_odom_pov"));
	    
	    return;
	}

}; // End of the PlanarCoordinator() Class



//--------------------------------------------------------------------------
// Main
//--------------------------------------------------------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planar_system_coordinator");

    // log4cxx::LoggerPtr my_logger =
    // 	log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(
    // 	ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle node;

    PlanarCoordinator planar_coordinator;
    ros::spin();

    return 0;
}



