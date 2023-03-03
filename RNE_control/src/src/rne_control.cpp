#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <cstdlib>  
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

static constexpr int ROS_RATE = 50;
static geometry_msgs::TwistStamped comman_vs;
static bool flag_detcect = false; 
static constexpr int THROSHOLD = 5;

static Eigen::Vector3d commands_vs= Eigen::MatrixXd::Zero(3, 1);
static Eigen::VectorXf setpoint_traj= Eigen::MatrixXf::Zero(6, 1);

static Eigen::Matrix3d R02d;
static Eigen::Matrix3d Rd2c;


void dronePoseCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) 
{
	Eigen::Quaterniond q_02d(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z);

    // normalise
    q_02d.normalize();

    // quaterion to rotation matrix
    R02d = q_02d.toRotationMatrix(); 
}


// callback function to receive detection mes from /tag_detections
void tagdetectCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    int number_tag_detected = msg->detections.size();

    if (number_tag_detected==1)
    {
        ROS_INFO_STREAM_THROTTLE(1,"tag is detected");
        flag_detcect = true;
    }
    else
    {
        ROS_INFO_STREAM_THROTTLE(1,"Tag is LOST");
        flag_detcect = false;
    }
    

}

// callback function to receive Visual Servoing mes from /visual_servoing_setpoint
void vsinputCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    Eigen::Vector3d commands_vs_cf(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);

    // commands_vs[0] = msg->twist.linear.x;
    // commands_vs[1] = msg->twist.linear.y;
    // commands_vs[2] = msg->twist.linear.z;
    Rd2c =  Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitY());
    commands_vs = R02d * Rd2c * commands_vs_cf;
    // std::cout<<commands_vs_cf.transpose()<<std::endl;
    // std::cout<<Rd2c<<std::endl;
    // std::cout<<R02d<<std::endl;
    ROS_INFO_STREAM_THROTTLE(2, "received vs setpoint" << commands_vs.transpose());
}

// callback function to receive trajectory setpoints mes from /reference/setpoint
void trajCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    setpoint_traj[0] = msg->twist.angular.x;
    setpoint_traj[1] = msg->twist.angular.y;
    setpoint_traj[2] = msg->twist.angular.z;
    setpoint_traj[3] = msg->twist.linear.x;
    setpoint_traj[4] = msg->twist.linear.y;
    setpoint_traj[5] = msg->twist.linear.z;    
    ROS_INFO_STREAM_THROTTLE(2, "received trajectory setpoint" << setpoint_traj.transpose());

}


int main(int argc, char* argv[])
{

    ros::init(argc, argv, "rne_visual_servoing");

    // define ros handle
    ros::NodeHandle nh;

    ros::Rate loop_rate(ROS_RATE);

    ros::Subscriber tag_sub = nh.subscribe("tag_detections", 1, tagdetectCallback);
    ros::Subscriber vs_sub = nh.subscribe<geometry_msgs::TwistStamped>("visual_servoing_setpoint", 1, vsinputCallback);
    // TODO choose topic
    ros::Subscriber traj_sub = nh.subscribe<geometry_msgs::TwistStamped>("trajectory/setpoint", 1, trajCallback);

    // choose topic
    ros::Subscriber dronepose_sub = nh.subscribe("vicon/drone", 1, dronePoseCallback);

    ros::Publisher  setpoint_pub = nh.advertise<geometry_msgs::TwistStamped>("reference/setpoint", 1);
    

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
            ROS_INFO_STREAM_THROTTLE(2, "Trajectory setpoint is sent as " << setpoint_traj.transpose());  

            setpoint_control.twist.angular.x = setpoint_traj[0];
            setpoint_control.twist.angular.y = setpoint_traj[1];
            setpoint_control.twist.angular.z = setpoint_traj[2];

            setpoint_control.twist.linear.x = setpoint_traj[3];
            setpoint_control.twist.linear.y = setpoint_traj[4];
            setpoint_control.twist.linear.z = setpoint_traj[5];
        }
        else
        {
            ROS_INFO_STREAM_THROTTLE(2,"Visual servoing setpoint is sent");
            // test
            // setpoint_control.twist.angular.x = setpoint_traj[0];
            // setpoint_control.twist.angular.y = setpoint_traj[1];
            // setpoint_control.twist.angular.z = setpoint_traj[2];

            // setpoint_control.twist.linear.x = setpoint_traj[3];
            // setpoint_control.twist.linear.y = setpoint_traj[4];
            // setpoint_control.twist.linear.z = setpoint_traj[5];
            
            // vs servoing
            setpoint_control.twist.angular.x = 0;
            setpoint_control.twist.angular.y = 0;
            setpoint_control.twist.angular.z = setpoint_traj[2];

            setpoint_control.twist.linear.x = commands_vs[0];
            setpoint_control.twist.linear.y = commands_vs[1];
            setpoint_control.twist.linear.z = setpoint_traj[5];

            ROS_INFO_STREAM_THROTTLE(1, "vel command x dire is " << commands_vs[0]);
            ROS_INFO_STREAM_THROTTLE(1, "vel command y dire is " << commands_vs[1]);

            // posi
            // setpoint_control.twist.angular.x = commands_vs[0];
            // setpoint_control.twist.angular.y = commands_vs[1];
            // setpoint_control.twist.angular.z = setpoint_traj[2];

            // setpoint_control.twist.linear.x = 0;
            // setpoint_control.twist.linear.y = 0;
            // setpoint_control.twist.linear.z = setpoint_traj[5];

            // ROS_INFO_STREAM("pos command x dire is " << commands_vs[0]);
            // ROS_INFO_STREAM("pos command y dire is " << commands_vs[1]);


            // ros::param::set("/another_integer", 0);
            // ros::param::set("/another_integer", 0);
            // // set position to be 0
            // setpoint_control.twist.angular.x = 0;
            // setpoint_control.twist.angular.y = 0;
            // setpoint_control.twist.angular.z = setpoint_traj[2];   


            // setpoint_control.twist.linear.x = commands_vs[0];
            // setpoint_control.twist.linear.y = commands_vs[1];
            // setpoint_control.twist.linear.z = commands_vs[2];
        }

        // ROS_INFO_STREAM("sentpoint to publish is "<< setpoint_traj.transpose());
        setpoint_pub.publish(setpoint_control);
        
        if (flag_detcect == true)
        {
            num_detcect ++;
            ROS_INFO_STREAM_THROTTLE(1, "Num of detecting tag is increasing");
        }
        else
        {
            num_detcect= 0;
            ROS_WARN_STREAM_THROTTLE(1, "Num of detecting tag is set to ZERO");
        }
        

        seq++;

    }
    
    return 0;
}
