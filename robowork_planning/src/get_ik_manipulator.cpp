#include <moveit/move_group/node_name.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <boost/scoped_ptr.hpp>
#include <tf/transform_listener.h>
// #include <octomap_msgs/conversions.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// #include <unordered_map>
// #include <queue>

// namespace move_group_interface {
template <typename T>
void println(T item)
{
  std::cout<< item <<std::endl;
}


void PrintTrajectory(moveit_msgs::RobotTrajectory& traj)
{
  std::vector<int>::size_type size1 = traj.joint_trajectory.points.size();
  ROS_INFO("Number of points = %i", size1);

  // std::vector<float> position;
  float position;
  //position.resize(vector1);
  int k = 0;
  for (unsigned i=0; i<size1; i++)
  {
    ROS_INFO("Trajectory at point %d",i);
    std::vector<int>::size_type size2 = traj.joint_trajectory.points[i].positions.size();
    for (unsigned j=0; j<size2; j++)
    { 
      position = traj.joint_trajectory.points[i].positions[j]; // positions[j]was eerst [0]
      if(j==0)
      {
        ROS_INFO("Joint angle for shoulder_pan_joint = %g", position);
      }
      else if(j==1)
      {
        ROS_INFO("Joint angle for shoulder_lift_joint = %g", position);
      }
      else if(j==2)
      {
        ROS_INFO("Joint angle for elbow_joint = %g", position);
      }
      else if(j==3)
      {
        ROS_INFO("Joint angle for wrist_1_joint = %g", position);
      }
      else if(j==4)
      {
        ROS_INFO("Joint angle for wrist_2_joint = %g", position);
      }
      else if(j==5)
      {
        ROS_INFO("Joint angle for wrist_3_joint = %g", position);
      }
      else
      {
        ROS_INFO("Joint angle for some random joint = %g", position);
      }
    }
  } 
}


// Eigen::Affine3d JointSpaceToCartesianPose(const trajectory_msgs::JointTrajectoryPoint& Traj, std::unique_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_, const robot_state::JointModelGroup* joint_model_group_)
// {
//   robot_model::RobotModelConstPtr robot_model  = move_group_->getRobotModel();
//   robot_state::RobotState *kinematic_state = new robot_state::RobotState(robot_model);
//   const std::string GRIPPER_NAME = move_group_->getEndEffectorLink();

//   std::vector<double> joint_values;
//   kinematic_state->copyJointGroupPositions(joint_model_group_, joint_values);
//   std::size_t size_joints = joint_values.size();
//   ROS_INFO_STREAM("existing number of joints = "<< size_joints);

//   std::vector<double> home_joint_values;
//   for(auto& x:Traj.positions)
//   {
//     home_joint_values.push_back((double)x);
//   }
//   kinematic_state->setJointGroupPositions(joint_model_group_, home_joint_values);
//   Eigen::Affine3d end_effector_state = kinematic_state->getGlobalLinkTransform("bvr_SIM/main_arm_SIM/gripper_manipulation_link");
//   println("Name of end effector: "+GRIPPER_NAME);

//   Eigen::Quaterniond q_pos(end_effector_state.rotation());
//   auto tr = end_effector_state.translation();

// //   println("##########################################################");
// //   ROS_INFO_STREAM("Translation:\r\n" << end_effector_state.translation());
// //   // ROS_INFO_STREAM("Rotation:\r\n" << end_effector_state.rotation());
// //   ROS_INFO_STREAM("Qauternion xyzw:\r\n" << q_pos.vec()<< "\n" << q_pos.w());
// //   // ROS_INFO_STREAM("Qauternion xyz:\r\n" );
// //   println("##########################################################");
//   delete kinematic_state;
// }


std::string PLANNING_GROUP_;
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
// std::unique_ptr<robot_model::RobotModelPtr> robot_model_;
// std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
std::unique_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
std::unique_ptr<tf::TransformListener> tf_listener_;

const robot_state::JointModelGroup* joint_model_group_;



visualization_msgs::Marker getMarker(const Eigen::Vector3d& position, const Eigen::Vector3d& color, uint action = 0, const std::string& ns = "points")
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    if(action == 0)
    {
        marker.action = visualization_msgs::Marker::ADD;
    }
    else if(action == 3)
    {
        marker.action = visualization_msgs::Marker::DELETEALL;
    }
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = color.x();
    marker.color.g = color.y();
    marker.color.b = color.z();
    marker.lifetime = ros::Duration(0.0);
    return marker;
}

visualization_msgs::MarkerArray getMarkerArray(const std::vector<Eigen::Vector3d>& position_list, const Eigen::Vector3d& color = Eigen::Vector3d(1.0,0.0,0.0), uint action = 0, const std::string& ns = "points")
{
    visualization_msgs::MarkerArray arr;
    for(auto position : position_list)
    {
        visualization_msgs::Marker marker = getMarker(position, color, action, ns);
        arr.markers.push_back(marker);
    }
    return arr;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_launch");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    PLANNING_GROUP_ = "main_arm_SIM";
    std::string robot_namespace = "bvr_SIM";
    std::string arm_namespace = "main_arm_SIM";
    node_handle.getParam("robot_namespace", robot_namespace);
    node_handle.getParam("arm_namespace", arm_namespace);
    std::string bvr_base_interia_frame = robot_namespace+"/bvr_base_inertia";
    std::string gripper_link = robot_namespace+"/main_arm_SIM/gripper_manipulation_link";
    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_);
    // planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
    robot_model::RobotModelConstPtr robot_model_ = move_group_->getRobotModel();
    robot_state::RobotStatePtr robot_state_(new robot_state::RobotState(robot_model_));
    joint_model_group_ = robot_state_->getJointModelGroup(PLANNING_GROUP_);

    // planning_scene::PlanningScenePtr planning_scene_(new planning_scene::PlanningScene(robot_model_));
    robot_model_loader::RobotModelLoader robot_model_loader_(robot_namespace+"/robot_description");
    planning_scene_monitor_ = std::make_unique<planning_scene_monitor::PlanningSceneMonitor>(robot_namespace+"/robot_description");
    planning_scene::PlanningScenePtr planning_scene_ = planning_scene_monitor_->getPlanningScene();
    bool success = planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
    ROS_INFO_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));
    //
    planning_scene_monitor_->startSceneMonitor(robot_namespace+"/move_group/monitored_planning_scene");

    

    move_group_->setMaxVelocityScalingFactor(0.25);
    // Set a scaling factor for optionally reducing the maximum joint acceleration.
    move_group_->setMaxAccelerationScalingFactor(1.0);
    // Planning with constraints can be slow because every sample must call an inverse kinematics solver (default 5 seconds)
    move_group_->setPlanningTime(5.0); //5.0
    // Number of times the motion plan is to be computed from scratch before the shortest solution is returned. 
    move_group_->setNumPlanningAttempts(100); //10	
    // Set the tolerance that is used for reaching the goal. For joint state goals, this will be distance for each joint, in the configuration space (radians or meters depending on joint type). For pose goals this will be the radius of a sphere where the end-effector must reach.
    move_group_->setGoalTolerance(0.02);
    // Pick one of the available configs - see ee ompl_planning<_SIM>.yaml for a complete list
    move_group_->setPlannerId("RRTstarkConfigDefault");

    // Configure a valid robot state
    planning_scene_->getCurrentStateNonConst().setToDefaultValues(joint_model_group_, "ready");

    // Visualization
    namespace rvt = rviz_visual_tools;
    // moveit_visual_tools::MoveItVisualTools visual_tools(bvr_base_interia_frame); //bvr_SIM/bvr_base_link
    moveit_visual_tools::MoveItVisualTools visual_tools("map");
    // visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();
    visual_tools.loadRemoteControl();

    visual_tools.prompt("Press 'next' to proceed");
    Eigen::Isometry3d end_effector_state = Eigen::Isometry3d::Identity();
    end_effector_state = robot_state_->getGlobalLinkTransform(robot_namespace+ "/"+ arm_namespace + "/gripper_manipulation_link");

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    visual_tools.prompt("Press 'next' to proceed");

    double timeout = 0.1;
    Eigen::Vector3d t;
    Eigen::Matrix3d r;
    r<< 0.7556335, -0.2257564,  0.6148593, 0.6548485,  0.2405590, -0.7164530, 0.0138339,  0.9440156,  0.3296107;
    t<< 0.3, 0.6, 0.9;
    end_effector_state.translation() = t;
    bool found_ik = robot_state_->setFromIK(joint_model_group_, end_effector_state, timeout);
    if (found_ik)
    {
        std::vector<double> joint_values;
        robot_state_->copyJointGroupPositions(joint_model_group_, joint_values);
        for (std::size_t i = 0; i < joint_values.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_values[i]);
        }
    }
    else
    {
        ROS_INFO_STREAM("Did not find IK solution for end effector state: " << end_effector_state.translation() << "\n"<<end_effector_state.rotation());
    }
        

    return 0;
}
