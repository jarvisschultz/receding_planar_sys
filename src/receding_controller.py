#!/usr/bin/env python
"""
Jarvis Schultz
Feb 2014

This node runs a true receding horizon optimization.


SUBSCRIPTIONS:
    - meas_config (PlanarSystemConfig)

PUBLISHERS:
    - filt_config (PlanarSystemConfig)
    - ref_config (PlanarSystemConfig)
    - serial_commands (RobotCommands)
    - mass_ref_path (Path)
    - mass_filt_path (Path)
    - post_covariance (PlanarCovariance)
    - filt_state (PlanarSystemState)
    - start_time (Time)

SERVICES:
    - get_ref_config (PlanarSystemService) (provider)

PARAMS:
    - window_length ~ defines number of timesteps in an optimization
    - controller_freq ~ defines the rate that everything is happening at
    - time_final ~ defines the total time to run the controller
"""

# ROS imports:
import roslib; roslib.load_manifest('receding_planar_sys')
import rospy
import tf
from std_msgs.msg import Time
from puppeteer_msgs.msg import FullRobotState
from puppeteer_msgs.msg import PlanarSystemConfig
from puppeteer_msgs.msg import RobotCommands
from puppeteer_msgs.msg import PlanarCovariance
from puppeteer_msgs.msg import PlanarSystemState
from puppeteer_msgs.srv import PlanarSystemService
from puppeteer_msgs.srv import PlanarSystemServiceRequest
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# local imports
import optimizer as op
import reference_manager as rm
import system_definition as sd
import ekf


# global constants:
DT = 1/10. # timestep
LEN =  20 # number of time steps to optimize over
Tper = rm.Tper


class RecedingController:

    def __init__(self):
        rospy.loginfo("Creating RecedingController class...")

        # first let's get all necessary parameters:
        self.get_and_set_params()

        # create a service provider for the reference configuration
        self.config_serv = rospy.Service("get_ref_config", PlanarSystemService,
                                            self.ref_config_service_handler)
        


    def ref_config_service_handler(self, req):
        



    def get_and_set_params(self):
        # controller frequency
        if rospy.has_param("controller_freq"):
            tmp = rospy.get_param("controller_freq")
            self.dt = 1/float(tmp)
        else:
            tmp = 1/DT
            self.dt = DT
            rospy.set_param("controller_freq", tmp)
        rospy.loginfo("Controller frequency = %f Hz (dt = %f sec.)", tmp, self.dt)

        # window length
        if rospy.has_param("window_length"):
            tmp = rospy.get_param("window_length")
            self.n_win = int(tmp)
        else:
            self.n_win = LEN
            rospy.set_param("window_length", self.n_win)
        rospy.loginfo("Window Length: %d",self.n_win)

        # controller run time
        if rospy.has_param("time_final"):
            tmp = rospy.get_param("time_final")
            self.tf = float(tmp)
        else:
            self.n_win = Tper
            rospy.set_param("time_final", self.n_win)
        rospy.loginfo("Final Time: %d",self.)
        


        





def main():
    """
    Run the main loop, by instatiating a System class, and then
    calling ros.spin
    """
    rospy.init_node('cl_control', log_level=rospy.INFO)

    try:
        controller = RecedingController()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
