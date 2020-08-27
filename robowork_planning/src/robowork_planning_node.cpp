# include <ros/ros.h>

#include <robowork_planning/MGI_AprilTag.h>

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "robowork_planning");

    ros::AsyncSpinner spinner(2); // TODO: Examples have 1, but move_group fails with: "Didn't received robot state (joint angles) with recent timestamp within 1 seconds." (see: https://github.com/ros-planning/moveit/issues/868)
    spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    move_group_interface::MGI_AprilTag move_group_interface_AprilTag(nh, pnh, "map");
  
    ros::waitForShutdown();
    return 0;
}
