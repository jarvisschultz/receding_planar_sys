#!/usr/bin/env python
"""
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
"""
import rospy
import copy
from interactive_markers.interactive_marker_server import *
import visualization_msgs.msg as VM
import std_srvs.srv as SS
from geometry_msgs.msg import Pose as P
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from puppeteer_msgs.msg import OperatingCondition
import tf


# global constants
DT = 1/100.
MARKERWF = 'optimization_frame'
MARKERFRAME = 'mass_ref_frame'

def makeMarker( msg, color ):
    marker = Marker()
    marker.type = Marker.SPHERE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.1
    marker.color.g = 0.1
    marker.color.b = 0.1
    marker.color.a = 0.75
    if color == 'red':
        marker.color.r += 0.4
    elif color == 'blue':
        marker.color.b += 0.4
    elif color == 'green':
        marker.color.g += 0.4
    else:
        rospy.warn("Marker color not recognized!")
    return marker


def makeMarkerControl( msg , color ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeMarker(msg, color) )
    msg.controls.append( control )
    return control



class SingleController:
    def __init__(self, conframe, simframe, simpos=[0.0,]*3,
                 simquat=[0.0,]*4, simpose=None, color='green'):
        """
        conframe ~ the frame that we should publish to control the kinematic
              input
        simframe ~ the frame that the simpos and simquat point to
        simpos ~ nominal location of the kinematic config variable in trep
              simulation... used for determining offset
        simquat ~ nominal orientation of the kinematic config var in the trep
              simulation... used for offset
        simpose ~ a pose message to define the pose
        color ~ color of the marker attached to the frame
        """
        self.conframe = conframe
        self.simframe = simframe
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = MARKERWF
        if simpose != None:
            self.int_marker.pose = simpose
        else:
            # build pose from pos and quat
            simquat[-1] = 1.0
            self.set_pose(pos=simpos, quat=simquat)
        self.int_marker.scale = 0.25
        self.int_marker.name = simframe
        self.int_marker.description = "set mass reference"

        # self.marker = makeMarker(self.int_marker, color)
        # self.marker.id = hash(conframe+simframe)%(2**16)
        # self.marker.header.frame_id = MARKERWF

        makeMarkerControl(self.int_marker, color)

        self.control = InteractiveMarkerControl()
        self.control.orientation.w = 1
        self.control.orientation.x = 0
        self.control.orientation.y = 1
        self.control.orientation.z = 0
        self.control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        self.int_marker.controls.append(self.control)


    def set_pose(self, pose=None, pos=[0.0,]*3, quat=[0.0,]*4):
        if pose != None:
            self.int_marker.header.frame_id = pose.header.frame_id
            self.int_marker.pose = pose
            self.simpose = copy.deepcopy(self.int_marker.pose)
            self.simpos = tuple([pose.position.__getattribute__(x)
                                 for x in pose.position.__slots__])
            self.simquat = tuple([pose.orientation.__getattribute__(x)
                                  for x in pose.orientation.__slots__])
        else:
            quat[-1] = 1
            self.simpos = pos
            self.simquat = quat
            self.int_marker.pose = P(position=Point(*pos),
                                     orientation=Quaternion(*quat))
            self.simpose = P(position=Point(*pos),
                                     orientation=Quaternion(*quat))


class MarkerControls:
    def __init__(self):
        # create marker server:
        self.server = InteractiveMarkerServer("mass_reference_control", q_size=1)
        # create listener and broadcaster
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()

        # let's build all of the controllers
        self.controllers = []
        self.controllers.append(SingleController(MARKERFRAME,
                                                 MARKERWF,
                                                 color='green'))

        # create subscriber for operating condition
        self.op_cond_sub = rospy.Subscriber("/operating_condition",
                                            OperatingCondition, self.opcb)
        self.operating_condition = OperatingCondition.IDLE
        # create publisher
        self.marker_pub = rospy.Publisher("mass_ref_point", PointStamped, queue_size=3)
        # publish markers for drawing paths
        self.con_pub = rospy.Publisher("visualization_markers", VM.MarkerArray, queue_size=3)
        # setup timer to publish transforms and messages:
        rospy.Timer(rospy.Duration(DT), self.timercb)
        return


    def opcb(self, data):
        if data.state < self.operating_condition:
            rospy.loginfo("Resetting interactive marker!")
            # reset marker:
            for con in self.controllers:
                con.set_pose()
                self.server.setPose(con.int_marker.name, con.simpose)
            self.server.applyChanges()
        self.operating_condition = data.state
        if self.operating_condition == OperatingCondition.RUN:
            for con in self.controllers:
                self.server.insert(con.int_marker, self.marker_cb)
            self.server.applyChanges()
        elif self.operating_condition == OperatingCondition.IDLE:
            rospy.loginfo("Removing interactive marker from server")
            for con in self.controllers:
                self.server.erase(con.int_marker.name)
            self.server.applyChanges()
        return


    def marker_cb(self, feedback):
        if self.operating_condition == OperatingCondition.RUN:
            rospy.loginfo("marker_cb called... type = {0:d}".format(feedback.event_type))
            if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
                print "x = {0:f}\r\ny = {1:f}\r\n\r\n".format(feedback.pose.position.x,
                                                              feedback.pose.position.y)
            self.server.applyChanges()
        return


    def send_transforms(self):
        tnow = rospy.Time.now()
        mlist = []
        for con in self.controllers:
            pos = con.int_marker.pose.position
            quat = con.int_marker.pose.orientation
            self.br.sendTransform((pos.x, pos.y, pos.z),
                                  (quat.x, quat.y, quat.z, quat.w),
                                  tnow,
                                  con.conframe, con.simframe)
            pt = PointStamped()
            pt.header.stamp = tnow
            pt.header.frame_id = MARKERWF
            pt.point.x = pos.x
            pt.point.y = pos.y
            pt.point.z = pos.z
            self.marker_pub.publish(pt)
            m = con.int_marker.controls[0].markers[0]
            m.header = con.int_marker.header
            m.pose = con.int_marker.pose
            mlist.append(m)
        ma = VM.MarkerArray()
        ma.markers = mlist
        self.con_pub.publish(ma)
        return


    def timercb(self, event):
        # check operating condition:
        if self.operating_condition == OperatingCondition.RUN:
            self.send_transforms()
        return


def main():
    rospy.init_node('marker_controls')

    try:
        sim = MarkerControls()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
