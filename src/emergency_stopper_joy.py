#!/usr/bin/env python
"""
Jarvis Schultz
May 2014

This node listens to a joystick input, and calls a service to go into emergency
stop if we detect an event

SUBSCRIPTIONS:
    - joy (sensor_msgs/Joy)

SERVICES:
    - operating_condition_change (OperatingConditionChange) (client)
"""


import roslib; roslib.load_manifest('receding_planar_sys')
import rospy
from puppeteer_msgs.msg import OperatingCondition
from puppeteer_msgs.srv import OperatingConditionChange
from puppeteer_msgs.srv import OperatingConditionChangeRequest
from sensor_msgs.msg import Joy

MAX_FREQ = 5 # max rate to print warnings:

class JoystickEmergency:
    def __init__(self):
        # wait for service servers:
        rospy.loginfo("Joystick emergency waiting for operating_condition_change service")
        rospy.wait_for_service("/operating_condition_change")
        rospy.loginfo("Joystick emergency detects operating_condition_change service now available")
        self.op_change_client = rospy.ServiceProxy("/operating_condition_change",
                                                   OperatingConditionChange)

        # create a subscriber to joystick topic:
        self.last_time = rospy.Time.now()
        self.joy_sub = rospy.Subscriber("joy", Joy, self.joycb)
        return
        
    def joycb(self, data):
        if (data.header.stamp-self.last_time).to_sec() > 1/float(MAX_FREQ):
            rospy.logwarn("Joystick event... stopping system!")
            self.last_time = data.header.stamp
        try:
            self.op_change_client(OperatingCondition(OperatingCondition.EMERGENCY))
        except rospy.ServiceException, e:
            rospy.loginfo("Joystick client: Service did not process request: %s"%str(e))
        return


def main():
    rospy.init_node('joystick_node', log_level=rospy.INFO)

    try:
        je = JoystickEmergency()
    except rospy.ROSInterruptException: pass

    rospy.spin()



if __name__ == '__main__':
    main()
