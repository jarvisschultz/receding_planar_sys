#!/usr/bin/env python
################
# ROS IMPORTS: #
################
import roslib; roslib.load_manifest('receding_planar_sys')
import rospy
import tf
import geometry_msgs.msg as GM
import visualization_msgs.msg as VM

FREQ_DIV = 3
NUMBER_MESSAGES = 200/float(FREQ_DIV)

class Path:
    def __init__(self, m):
        self.marker = VM.Marker()
        self.marker.type = VM.Marker.LINE_STRIP
        scale = m.scale.x/10.0
        self.marker.scale = GM.Vector3(*(scale, scale, scale))
        self.marker.header.frame_id = m.header.frame_id
        self.marker.id = hash(m.id + self.marker.type)%(2**16)
        self.marker.color = m.color
        self.marker.color.a = 1.0
        self.point_list = []
        self.max_size = rospy.get_param('path_len', NUMBER_MESSAGES)
        self.update_path(m)


    def update_path(self, m):
        self.point_list.append(m.pose.position)
        if len(self.point_list) > self.max_size:
            self.point_list.pop(0)
        self.marker.points = self.point_list
        


class MarkerPaths:
    def __init__(self):
        # subscriber for markers
        self.mark_sub = rospy.Subscriber("visualization_markers", VM.MarkerArray,
                                         self.markcb)
        # publisher for the paths of the markers
        self.path_pub = rospy.Publisher("marker_paths", VM.MarkerArray)
        self.paths = {}
        self.count = 0

    def markcb(self, msg):
        self.count += 1
        if self.count%FREQ_DIV == 0:
            # update all of the marker paths
            for m in msg.markers:
                if m.id not in self.paths.keys() and m.type == VM.Marker.SPHERE:
                    self.paths[m.id] = Path(m)
                elif m.id in self.paths.keys():
                    self.paths[m.id].update_path(m)
            # now we can assemble a MarkerArray and publish:
            ma = VM.MarkerArray()
            ma.markers = [a.marker for a in self.paths.values()]
            self.path_pub.publish(ma)
        return

def main():
    rospy.init_node('marker_path_pub')
    try:
        sim = MarkerPaths()
    except rospy.ROSInterruptException: pass

    rospy.spin()


if __name__=='__main__':
    main()
