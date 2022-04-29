import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
import argparse
import csv
from visualization_msgs.msg import Marker

def get_mesh(filename):
    mesh = Marker()
    mesh.type = Marker.MESH_RESOURCE
    mesh.mesh_resource = filename
    mesh.scale.x = 1
    mesh.scale.y = 1
    mesh.scale.z = 1
    mesh.color.r = 0
    mesh.color.g = 0
    mesh.color.b = 1
    mesh.color.a = 1
    mesh.header.frame_id = 'map'
    mesh.header.stamp = rospy.Time.now()
    return mesh

def get_normals(csv_file):
    pose_arr = PoseArray()
    pose_arr.header.frame_id = 'map'
    pose_arr.header.stamp = rospy.Time.now()
    centers = []
    quat = []
    with open(csv_file) as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            if row[0] == 'x':
                continue
            # print(row)
            # print(row[0])
            row = np.array(row, dtype=np.float64)
            center = np.zeros((3,),dtype=np.float64)
            q = np.zeros((4,),dtype=np.float64)
            center[0] = float(row[0])
            center[1] = float(row[1])
            center[2] = float(row[2])
            q[0] = float(row[3])
            q[1] = float(row[4])
            q[2] = float(row[5])
            q[3] = float(row[6])
            centers.append(center)
            quat.append(q)
    centers = np.array(centers)
    quat = np.array(quat)
    # print("centers max = {}".format(np.amax(centers,axis=0)))
    # print("centers min = {}".format(np.amin(centers,axis=0)))
    for i in range(len(centers)):
        pose = Pose()
        pose.position.x = centers[i,0]
        pose.position.y = centers[i,1]
        pose.position.z = centers[i,2]
        pose.orientation.x = quat[i,0]
        pose.orientation.y = quat[i,1]
        pose.orientation.z = quat[i,2]
        pose.orientation.w = quat[i,3]
        pose_arr.poses.append(pose)
    return pose_arr

def main():
    rospy.init_node('publish_normal')
    parser = argparse.ArgumentParser(description='Publish normal vectors as quaternions for visualization in RViz')
    parser.add_argument('-c','--csv_file', type=str, help='csv file containing normal vectors', default='boat.csv')
    parser.add_argument('-m','--mesh_file', type=str, help='mesh file to visualize normals')
    args = parser.parse_args()

    pub = rospy.Publisher('/pose', PoseArray, queue_size=10)
    mesh_pub = rospy.Publisher('/mesh', Marker, queue_size=10)
    rate = rospy.Rate(1)
    poses = get_normals(args.csv_file)
    # mesh = get_mesh('/home/prateek/boat.dae')
    print("publishing normals as posearray")
    while not rospy.is_shutdown():
        # mesh_pub.publish(mesh)
        pub.publish(poses)
        rate.sleep()

if __name__ == '__main__':
    main()
