#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/Int32.h"

// Define a client for sending goal requests to the move_base server through a
// SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

int main(int argc, char **argv)
{
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    ROS_INFO("node inited");

    ros::NodeHandle n;

    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // wait 5 seconds for the move_base action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("waiting for the move_base action server to come up");
    }

    ROS_INFO("action server is up");

    // add a publisher to tell if the robot has reached the goal
    // 1 == op started, 2 == pickup goal, 3 == dropoff goal, 4 == op failed
    ros::Publisher arrived_pub = n.advertise<std_msgs::Int32>("arrived_flag",
                                                              1000);
    std_msgs::Int32 flag;

    move_base_msgs::MoveBaseGoal pickup_goal;
    move_base_msgs::MoveBaseGoal dropoff_goal;

    // set up the frame parameters
    pickup_goal.target_pose.header.frame_id = "map";  // this means the goal
                                                      // position will be
                                                      // absolute
    pickup_goal.target_pose.header.stamp    = ros::Time::now();

    dropoff_goal.target_pose.header.frame_id = "map"; // this means the goal
                                                      // position will be
                                                      // absolute
    dropoff_goal.target_pose.header.stamp    = ros::Time::now();

    // Define a position and orientation for the robot to reach
    pickup_goal.target_pose.pose.position.x    = -0.61;
    pickup_goal.target_pose.pose.position.y    = -2.65;
    pickup_goal.target_pose.pose.orientation.z = 0.99;
    pickup_goal.target_pose.pose.orientation.w = -0.13;

    dropoff_goal.target_pose.pose.position.x    = -6.03;
    //dropoff_goal.target_pose.pose.position.y    = 2.1;
    dropoff_goal.target_pose.pose.position.y    = -2.91;
    dropoff_goal.target_pose.pose.orientation.z = 0.556;
    dropoff_goal.target_pose.pose.orientation.w = 0.83;

    // send the pickup goal position and orientation for the robot to reach
    ROS_INFO("Sending pickup goal");

    flag.data = 1; // robot started
    arrived_pub.publish(flag);
    ROS_INFO("robot started flag raised");

    ac.sendGoal(pickup_goal);

    ROS_INFO("waiting for the robot to go to pick up goal");

    // wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot has reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Hooray, the base moved to the pick up goal");
        flag.data = 2; // reached the pick up goal
        arrived_pub.publish(flag);
        ROS_INFO("Pickup flag raised");
    }
    else
    {
        ROS_INFO("The base failed to move to the pick up goal for some reason");
        flag.data = 4; // failed to reach the pick up goal
        arrived_pub.publish(flag);
        ROS_INFO("Fail flag raised");
    }

    ROS_INFO("waiting 5 sec before moving to dropoff");
    ros::Duration(5.0).sleep();

    // send the dropoff goal position and orientation for the robot to reach
    ROS_INFO("Sending dropoff goal");
    ac.sendGoal(dropoff_goal);

    ROS_INFO("waiting for the robot to go to drop off goal");

    // wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot has reached its goal
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Hooray, the base moved to the drop off goal");
        flag.data = 3; // reached the dropoff goal
        arrived_pub.publish(flag);
        ROS_INFO("Dropoff flag raised");
    }
    else
    {
        ROS_INFO("The base failed to move to drop off goal for some reason");
        flag.data = 4; // failed to reach the pick up goal
        arrived_pub.publish(flag);
        ROS_INFO("Fail flag raised");
    }

    return 0;
}
