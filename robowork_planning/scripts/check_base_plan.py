import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import nav_msgs.msg
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped, Point

class baseIK(object):
    def __init__(self):
        rospy.loginfo("Initalizing baseIK...")
        self.frame_id = 'map'
        rospy.loginfo("PoseStamped answers will be on frame: " + self.frame_id)
        self.base_plan_srv = rospy.ServiceProxy('bvr_SIM/move_base/make_plan',GetPlan)
        rospy.loginfo("Waiting for /make_plan service...")
        self.base_plan_srv.wait_for_service()
        rospy.loginfo("Connected!")
        
    def get_plan(self, pos_start, pos_goal):
        req = GetPlanRequest()
        start = PoseStamped()
        goal = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = rospy.Time.now()
        start.pose.position = pos_start
        start.pose.orientation.w = 1.0

        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position = pos_goal
        goal.pose.orientation.w = 1.0

        req.start = start
        req.goal = goal
        req.tolerance = 0.5
        resp = self.base_plan_srv(req.start, req.goal, req.tolerance)
        
        return resp

def main():
   move_base = baseIK()
   rospy.init_node('check_base',anonymous=True)
   start = Point()
   start.x = 0.0
   start.y = 0.0
   start.z = 0.0
   goal = Point()
   goal.x = 0.0
   goal.y = -0.0
   goal.z = 0.0
   resp = move_base.get_plan(start,goal)
   print(resp)

if __name__ == "__main__":
    main()