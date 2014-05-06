#!/usr/bin/env python

import roslib; roslib.load_manifest("interactive_markers")
import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin
import select
import sys

server = None
marker_pos = 0
br = None
counter = 0

def frameCallback( msg ):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform( (0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time, "base_link", "moving_frame" )
    counter += 1

def processFeedback( feedback ):
    rospy.loginfo("processFeedback called")
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo( s + ": pose changed")
        print "x = {0:f}\r\ny = {1:f}\r\nz = {2:f}\r\n\r\n".format(feedback.pose.position.x,
                                                                   feedback.pose.position.y,
                                                                   feedback.pose.position.z)
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo( s + ": mouse down" + mp + "." )
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()


def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control


#####################################################################
# Marker Creation
def makeQuadrocopterMarker():
    global marker_pos
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/base_link"
    int_marker.pose.position.y = -3.0 * marker_pos
    int_marker.scale = 1

    int_marker.name = "quadrocopter"
    int_marker.description = "Quadrocopter"

    makeBoxControl(int_marker)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
    int_marker.controls.append(copy.deepcopy(control))
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)


def keyboardcb(event):
    while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline()
        if line:
            print "Input = ",line,
            if 'a' in line:
                print "Enable controls"
                makeQuadrocopterMarker()
                server.applyChanges()
            elif 'b' in line:
                print "Disable controls"
                for marker_name in server.marker_contexts.keys():
                    server.erase(marker_name)
                server.applyChanges()
            else:
                print "Unknown"
            print "\r\n",


if __name__=="__main__":
    rospy.init_node("basic_controls")
    br = TransformBroadcaster()
    
    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)
    
    # create a timer to check for keyboard input
    rospy.Timer(rospy.Duration(0.1), keyboardcb)

    server = InteractiveMarkerServer("basic_controls", q_size=1)

    # makeQuadrocopterMarker( )

    server.applyChanges()

    rospy.spin()

