#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <cstdlib>  
#include <Eigen/Dense>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/TwistStamped.h>


static constexpr int ROS_RATE = 50;
static const Eigen::Vector2f targ_position_desXY= Eigen::MatrixXf::Zero(2, 1);
static geometry_msgs::TwistStamped comman_vs;

// tag_position to store tag position in xy plane 
Eigen::Vector2f tag_position_XY = Eigen::MatrixXf::Zero(2, 1);

// callback function to receive detection mes from /tag_detections
void tagdetectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    int number_tag_detected = msg->detections.size();

    /* Tag is not detected-> skip*/
    // if no tag is detected, then skip this turn of callback function
    if (number_tag_detected != 1)
    {
        ROS_INFO_STREAM("No tag is detected");
        return; // skip this callback function since not all markers are detected
    }

    /* Tag is detected-> skip*/
    // 1. save tag position into tag_position
    tag_position_XY[0] = msg->detections[0].pose.pose.pose.position.x;
    tag_position_XY[1] = msg->detections[0].pose.pose.pose.position.y;


    // Debug: to print rag position
    ROS_INFO_STREAM("Tag in image plane"<<tag_position_XY.transpose());
    // TODO height

    // 2. Call P control
    float Kp = 2;
    Eigen::Vector2f u_vs= Eigen::MatrixXf::Zero(2, 1);

    // vel setpoint in XY plane
    u_vs = Kp*(targ_position_desXY-tag_position_XY);

    // 3. transfer vel setpoint to ros msg

    static unsigned int seq = 1;
    comman_vs.header.seq=seq;
    comman_vs.header.frame_id="base_link";
    comman_vs.twist.angular.x = 0;
    comman_vs.twist.angular.y = 0;
    comman_vs.twist.angular.z = 0;

    comman_vs.twist.linear.x = u_vs[0];
    comman_vs.twist.linear.y = u_vs[1];
    comman_vs.twist.linear.z = 0;
    
    seq++;
}


int main(int argc, char* argv[])
{

    ros::init(argc, argv, "rne_visual_servoing");

    // define ros handle
    ros::NodeHandle nh;

    ros::Rate loop_rate(ROS_RATE);


// DONE      1.1. get ariltag detection massage: position of tag in image frame
// DONE   1.2. save position into Eigen
// DONE       1.3. define subscriber to get detection infor
    ros::Subscriber sub = nh.subscribe("/tag_detections", 1, tagdetectCallback);
    ros::Publisher  vs_pub = nh.advertise<geometry_msgs::TwistStamped>("/visual_servoing_setpoint", 1);
    
//      2.1 find a pid controller from C++ library
//      2.2 call a controller to compute vel setpoint
// TODO 2.3 set gains and leave ros parameter to tune

// DONE 3.1  define a publisher for vel setpoint 
// DONE 3.2  transform controller output to setpoint
// DONE 3.3  pub   

    while (ros::ok()) {

        ros::spinOnce();
  
        loop_rate.sleep();

        // 4. pusb
        vs_pub.publish(comman_vs);

    }
    
    return 0;
}
