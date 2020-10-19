#!/usr/bin/env python2


import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
import numpy as np
import rospy


def readPoints():
    f = open("/home/prateek/arm.txt", "r")
    list_ = []
    while True:
        t = np.zeros((7,),dtype=np.float64)
        line1 = f.readline()
        # print(line1)
        if not line1:
            break
        line2 = f.readline()
        # print("line2: ",line2)
        arr = line2.split(":")
        # print("t0 : ",float(arr[-1]))
        t[0] = float(arr[-1])
        line3 = f.readline()
        # print("t1 : ",float(line3))
        t[1] = float(line3)
        line4 = f.readline()
        t[2] = float(line4)
        list_.append(t)
        line5 = f.readline()
        line6 = f.readline()
        t[3] = float(line6)
        line7 = f.readline()
        t[4] = float(line7)
        line8 = f.readline()
        t[5] = float(line8)
        line9 = f.readline()
        t[6] = float(line9)
    
    # print("elements are =")
    # for elem in list_:
    #     print(elem[0],elem[1],elem[2])

    return list_



def main():
    list_ = readPoints()
    topic = 'visualization_traj'
    publisher = rospy.Publisher(topic, MarkerArray, queue_size=5)

    rospy.init_node('register')

    markerArray = MarkerArray()
    count = 0
    MARKERS_MAX = len(list_)
    r = rospy.Rate(1)


    for elem in list_:
        marker = Marker()
        # marker.header.frame_id = "/bvr_SIM/bvr_base_inertia"
        # bvr_SIM/bvr_base_link
        marker.header.frame_id = "map"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.01
        marker.scale.z = 0.008
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.x = elem[5]
        marker.pose.orientation.y = elem[4]
        marker.pose.orientation.z = elem[3]
        marker.pose.orientation.w = elem[6]
        marker.pose.position.x = elem[0]
        marker.pose.position.y = elem[1]
        marker.pose.position.z = elem[2] 

        # We add the new marker to the MarkerArray, removing the oldest marker from it when necessary
        # if(count > MARKERS_MAX):
        #     markerArray.markers.pop(0)

        markerArray.markers.append(marker)

    # for marker in markerArray.markers:
    #     print(marker.pose.position)
    # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
        m.id = id
        id += 1

        # Publish the MarkerArray
    while not rospy.is_shutdown():
        publisher.publish(markerArray)

        count += 1

        r.sleep()


if __name__ == "__main__":
    main()
