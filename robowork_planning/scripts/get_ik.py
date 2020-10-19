#!/usr/bin/env python2

from __future__ import print_function

import sys
import rospy
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Header


class GetFK(object):
    def __init__(self, fk_link, frame_id):
        """
        A class to do FK calls thru the MoveIt!'s /compute_fk service.
        :param str fk_link: link to compute the forward kinematics
        :param str frame_id: frame_id to compute the forward kinematics
        into account collisions
        """
        rospy.loginfo("Initalizing GetFK...")
        self.fk_link = fk_link
        self.frame_id = frame_id
        rospy.loginfo("Asking forward kinematics for link: " + self.fk_link)
        rospy.loginfo("PoseStamped answers will be on frame: " + self.frame_id)
        self.fk_srv = rospy.ServiceProxy('bvr_SIM/compute_fk',
                                         GetPositionFK)
        rospy.loginfo("Waiting for /compute_fk service...")
        self.fk_srv.wait_for_service()
        rospy.loginfo("Connected!")
        self.last_js = None
        self.js_sub = rospy.Subscriber('bvr_SIM/joint_states',
                                       JointState,
                                       self.js_cb,
                                       queue_size=1)

    def js_cb(self, data):
        self.last_js = data

    def get_current_fk_pose(self):
        resp = self.get_current_fk()
        if len(resp.pose_stamped) >= 1:
            return resp.pose_stamped[0]
        return None

    def get_current_fk(self):
        while not rospy.is_shutdown() and self.last_js is None:
            rospy.logwarn("Waiting for a /joint_states message...")
            rospy.sleep(0.1)
        return self.get_fk(self.last_js)

    def get_fk(self, joint_state, fk_link=None, frame_id=None):
        """
        Do an FK call to with.
        :param sensor_msgs/JointState joint_state: JointState message
            containing the full state of the robot.
        :param str or None fk_link: link to compute the forward kinematics for.
        """
        if fk_link is None:
            fk_link = self.fk_link

        req = GetPositionFKRequest()
        req.header.frame_id = 'base_link'
        req.fk_link_names = [self.fk_link]
        req.robot_state.joint_state = joint_state
        try:
            resp = self.fk_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionFKResponse()
            resp.error_code = 99999  # Failure
            return resp



class GetIK(object):
    def __init__(self, group, ik_timeout=1.0, ik_attempts=0,
                 avoid_collisions=False):
        """
        A class to do IK calls thru the MoveIt!'s /compute_ik service.
        :param str group: MoveIt! group name
        :param float ik_timeout: default timeout for IK
        :param int ik_attempts: default number of attempts
        :param bool avoid_collisions: if to ask for IKs that take
        into account collisions
        """
        print("Initalizing GetIK...")
        self.group_name = group
        self.ik_timeout = ik_timeout
        self.ik_attempts = ik_attempts
        self.avoid_collisions = avoid_collisions
        print("Computing IKs for group: ", self.group_name)
        print("With IK timeout: ", str(self.ik_timeout))
        print("And IK attempts: ", str(self.ik_attempts))
        print("Setting avoid collisions to: " ,
                      str(self.avoid_collisions))
        self.ik_srv = rospy.ServiceProxy('bvr_SIM/compute_ik',GetPositionIK)
        print("Waiting for /compute_ik service...")
        self.ik_srv.wait_for_service()
        print("Connected!")

    def get_ik(self, pose_stamped,
               group=None,
               ik_timeout=None,
               ik_attempts=None,
               avoid_collisions=None):
        """
        Do an IK call to pose_stamped pose.
        :param geometry_msgs/PoseStamped pose_stamped: The 3D pose
            (with header.frame_id)
            to which compute the IK.
        :param str group: The MoveIt! group.
        :param float ik_timeout: The timeout for the IK call.
        :param int ik_attemps: The maximum # of attemps for the IK.
        :param bool avoid_collisions: If to compute collision aware IK.
        """
        if group is None:
            group = self.group_name
        if ik_timeout is None:
            ik_timeout = self.ik_timeout
        if ik_attempts is None:
            ik_attempts = self.ik_attempts
        if avoid_collisions is None:
            avoid_collisions = self.avoid_collisions
        req = GetPositionIKRequest()
        req.ik_request.group_name = group
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(ik_timeout)
        # req.ik_request.attempts = ik_attempts
        req.ik_request.avoid_collisions = avoid_collisions

        try:
            resp = self.ik_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionIKResponse()
            resp.error_code = 99999  # Failure
            return resp
def main():
    PLANNING_GROUP_ = "main_arm_SIM"
    rospy.init_node('get_ik_node',anonymous=True)
    # rospy.loginfo("node started")
    # print("planning group = ", PLANNING_GROUP_)
    
    rospy.loginfo("Querying for FK")
    gfk = GetFK('bvr_SIM/main_arm_SIM/gripper_manipulation_link', 'bvr_SIM/base_link')
    resp = gfk.get_current_fk()
    print("fk solution", resp.pose_stamped[0].pose)
    print("fk solution frame id", resp.pose_stamped[0].header.frame_id)
    
    
    
    ik_ob = GetIK(group=PLANNING_GROUP_)
    point3d = PoseStamped()
    point3d.header.frame_id = 'map'
    # point3d.header.frame_id = 'bvr_SIM/bvr_base_link'
    point3d.pose.position.x = 0.745
    point3d.pose.position.y = -0.13
    point3d.pose.position.z = 0.86
    point3d.pose.orientation.w = 0.999986455466
    point3d.pose.orientation.x = -0.000981840090629
    point3d.pose.orientation.y = 0.0049986801065
    point3d.pose.orientation.z = 0.00106680414674
    




    # print("point = ", point3d)
    
    response = ik_ob.get_ik(point3d)
    print("ik response code", response.error_code)
    if(response.error_code == 1):
        print("ik found\n")
    else:
        print("No solution found ")



if __name__ == "__main__":
    main()