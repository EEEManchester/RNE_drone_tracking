#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <cstdlib>  
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseStamped.h>


ros::Publisher pose_pub;

// callback function to receive detection mes from /tag_detections
void tagdetectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    int number_tag_detected = msg->detections.size();

    /* Tag is not detected-> skip*/
    // if no tag is detected, then skip this turn of callback function
    if (number_tag_detected != 1)
    {
        ROS_WARN_STREAM("No tag is detected");
        return; // skip this callback function since not all markers are detected
    }

    ROS_INFO_STREAM("Tag is detected");

    geometry_msgs::PoseStamped poseMavros;

    poseMavros.header.frame_id = "base_link";
    poseMavros.header.stamp = ros::Time::now();

    poseMavros.pose = msg->detections[0].pose.pose.pose;

    pose_pub.publish(poseMavros);
}


int main(int argc, char* argv[])
{

    // Start ROS
    ros::init(argc, argv, "tag_navigation");

    // create global node handle for publishing and subscribing
    ros::NodeHandle nh;
    // create local node handle for private parameters
    ros::NodeHandle nh_loc("~");

    // set loop rate of vicon
    int rate;
    nh_loc.param("navig_rate",rate, 50);
    ros::Rate loop_rate(rate);
    ROS_INFO("tag navigation spinning at %d Hz", rate);


    ros::Subscriber sub = nh.subscribe("/tag_detections", 1, tagdetectCallback);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 1);
    

    while (ros::ok()) {

        ros::spinOnce();
  
        loop_rate.sleep();

    }
    
    return 0;
}
