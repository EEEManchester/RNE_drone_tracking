#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <cstdlib>  
#include <Eigen/Dense>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/TwistStamped.h>


static constexpr int ROS_RATE = 30;
static geometry_msgs::TwistStamped comman_vs;
static bool flag_detcect = false; 
static constexpr int THROSHOLD = 50;

static Eigen::Vector3f commands_vs= Eigen::MatrixXf::Zero(3, 1);
static Eigen::VectorXf setpoint_traj= Eigen::MatrixXf::Zero(6, 1);


// callback function to receive detection mes from /tag_detections
void tagdetectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    int number_tag_detected = msg->detections.size();

    if (number_tag_detected==1)
    {
        ROS_INFO_STREAM("tag is detected");
        flag_detcect = true;
    }
    else
    {
        ROS_WARN_STREAM("Tag is LOST");
        flag_detcect = false;
    }
    

}

// callback function to receive Visual Servoing mes from /visual_servoing_setpoint
void vsinputCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{

    commands_vs[0] = msg->twist.linear.x;
    commands_vs[1] = msg->twist.linear.y;
    commands_vs[2] = msg->twist.linear.z;

}

// callback function to receive trajectory setpoints mes from /reference/setpoint
void traj_sub(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    setpoint_traj[0] = msg->twist.angular.x;
    setpoint_traj[1] = msg->twist.angular.y;
    setpoint_traj[2] = msg->twist.angular.z;
    setpoint_traj[3] = msg->twist.linear.x;
    setpoint_traj[4] = msg->twist.linear.y;
    setpoint_traj[5] = msg->twist.linear.z;    

}


int main(int argc, char* argv[])
{

    ros::init(argc, argv, "rne_visual_servoing");

    // define ros handle
    ros::NodeHandle nh;

    ros::Rate loop_rate(ROS_RATE);

    ros::Subscriber tag_sub = nh.subscribe("rne_control/tag_detections", 1, tagdetectCallback);
    ros::Subscriber vs_sub = nh.subscribe<geometry_msgs::TwistStamped>("rne_control/visual_servoing_setpoint", 1, vsinputCallback);
    // TODO choose topic
    ros::Subscriber traj_sub = nh.subscribe<geometry_msgs::TwistStamped>("rne_control/trajectory/setpoint", 1, vsinputCallback);

    ros::Publisher  setpoint_pub = nh.advertise<geometry_msgs::TwistStamped>("rne_control/reference/setpoint", 1);
    

    unsigned int num_detcect = 0;
    while (ros::ok()) {

        ros::spinOnce();
  
        loop_rate.sleep();
        

        static geometry_msgs::TwistStamped setpoint_control;
        static unsigned int seq = 1;

        static unsigned int num_detcect=0; 

        setpoint_control.header.seq=seq;
        setpoint_control.header.frame_id="base_link";
        setpoint_control.header.stamp = ros::Time::now();

        if (num_detcect<=THROSHOLD)
        {
            ROS_INFO_STREAM("Trajectory setpoint is sent");
            setpoint_control.twist.angular.x = setpoint_traj[0];
            setpoint_control.twist.angular.y = setpoint_traj[1];
            setpoint_control.twist.angular.z = setpoint_traj[2];

            setpoint_control.twist.linear.x = setpoint_traj[3];
            setpoint_control.twist.linear.y = setpoint_traj[4];
            setpoint_control.twist.linear.z = setpoint_traj[5];
        }
        else
        {
            ROS_INFO_STREAM("visual servoing control is sent");
            setpoint_control.twist.linear.x = commands_vs[0];
            setpoint_control.twist.linear.y = commands_vs[1];
            setpoint_control.twist.linear.z = commands_vs[2];
        }

        ROS_INFO_STREAM("sent point is "<< setpoint_traj.transpose());
        setpoint_pub.publish(setpoint_control);
        
        if (flag_detcect == true)
        {
            num_detcect ++;
            ROS_INFO_STREAM("Num of detecting tag is increasing");
        }
        else
        {
            num_detcect= 0;
            ROS_WARN_STREAM("Num of detecting tag is set to ZERO");
        }
        

        seq++;

    }
    
    return 0;
}
