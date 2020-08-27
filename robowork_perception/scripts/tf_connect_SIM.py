
#!/usr/bin/python

import rospy
import tf
import tf2_ros
from nav_msgs.msg import Odometry
import numpy

# def odometryCb(msg):
#   #frame_id: "map"
#   #child_frame_id: "bvr_SIM/bvr_base_link"
#   print msg.pose.pose

if __name__ == '__main__':
  rospy.init_node('tf_connect_SIM')
  listener = tf.TransformListener()
  broadcaster = tf.TransformBroadcaster()
  # rospy.Subscriber('odom', Odometry, odometryCb)

  rate = rospy.Rate(100.0)
  while not rospy.is_shutdown():
    ros_time_now = rospy.Time.now()

    #broadcaster.sendTransform((0,0,0), (0,0,0,1), ros_time_now, '/bvr_SIM/t265_link', '/bvr_SIM/main_arm_SIM/t265_link')

    try:
      camodom_to_cam_time = listener.getLatestCommonTime('/bvr_SIM/t265_odom_frame', '/bvr_SIM/t265_pose_frame')
      (camodom_to_cam_T, camodom_to_cam_R) = listener.lookupTransform('/bvr_SIM/t265_odom_frame', '/bvr_SIM/t265_pose_frame', camodom_to_cam_time)
      #print camodom_to_cam_T, camodom_to_cam_R

      listener.waitForTransform('/bvr_SIM/main_arm_SIM/t265_link', '/bvr_SIM/bvr_base_link', camodom_to_cam_time, rospy.Duration(0.1))
      (cam_to_base_T, cam_to_base_R) = listener.lookupTransform('/bvr_SIM/main_arm_SIM/t265_link', '/bvr_SIM/bvr_base_link', camodom_to_cam_time)
      #print cam_to_base_T, cam_to_base_R

      camodom_to_cam_T_mat = tf.transformations.translation_matrix(camodom_to_cam_T)
      camodom_to_cam_R_mat = tf.transformations.quaternion_matrix(camodom_to_cam_R)
      camodom_to_cam_mat = numpy.dot(camodom_to_cam_T_mat, camodom_to_cam_R_mat)
      cam_to_base_T_mat = tf.transformations.translation_matrix(cam_to_base_T)
      cam_to_base_R_mat = tf.transformations.quaternion_matrix(cam_to_base_R)
      cam_to_base_mat = numpy.dot(cam_to_base_T_mat, cam_to_base_R_mat)

      camodom_to_base_time = camodom_to_cam_time
      camodom_to_base_mat = numpy.dot(camodom_to_cam_mat, cam_to_base_mat)
      camodom_to_base_T = tf.transformations.translation_from_matrix(camodom_to_base_mat)
      camodom_to_base_R = tf.transformations.quaternion_from_matrix(camodom_to_base_mat)

      broadcaster.sendTransform(camodom_to_base_T, camodom_to_base_R, camodom_to_base_time, '/bvr_SIM/bvr_base_link', '/bvr_SIM/t265_odom_frame')
      #print camodom_to_base_T, camodom_to_base_R

    except tf.LookupException as e:
      print ('tf_connect: tf.LookupException', e)
      pass
    except tf.ConnectivityException as e:
      print ('tf_connect: tf.ConnectivityException', e)
      pass
    except tf.ExtrapolationException as e:
      print ('tf_connect: tf.ExtrapolationException', e)
      pass
    except tf2_ros.TransformException as e:
      print ('tf_connect: tf2_ros.TransformException', e)
      pass

    rate.sleep()
