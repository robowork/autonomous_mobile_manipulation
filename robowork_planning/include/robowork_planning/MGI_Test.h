#ifndef MGI_TEST_
#define MGI_TEST_

#include <moveit/move_group/node_name.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <sensor_msgs/Joy.h>

#include <tf/transform_listener.h>


namespace move_group_interface {

class MGI_Test {
  public:
    MGI_Test(ros::NodeHandle nh, ros::NodeHandle private_nh, std::string world_link="map")
      : nh_(nh)
      , private_nh_(private_nh)
      , world_link_(world_link)
    {
      initialize();
    }

  private:
    // ROS-related heavyweight setup
    void initialize();    

    // execute planning attempt
    void plan();    

    // rviz visual tools gui
    void rc_cb(const sensor_msgs::Joy::ConstPtr& msg);
    // goal for end-effector
    void endeffector_goal_cb(const geometry_msgs::PointStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    const std::string world_link_;
    std::string plan_link_;
    std::string ee_link_;

    std::string PLANNING_GROUP_;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::unique_ptr<tf::TransformListener> tf_listener_;

    const robot_state::JointModelGroup* joint_model_group_;

    tf::StampedTransform T_EE_W_;  // end-effector to world
    tf::Stamped<tf::Pose> T_EE_ref_W_{tf::Pose(), ros::Time(0), ""};  // end-effector-reference to world

    ros::Subscriber rc_sub_;
    ros::Subscriber endeffector_goal_sub_;

};  //  class MGI_Test

}  // namespace move_group_interface

#endif  // MGI_TEST_
