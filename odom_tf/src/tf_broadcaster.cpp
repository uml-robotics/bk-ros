#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>


void poseCallback(const nav_msgs::Odometry::ConstPtr& odomsg)
{
    extern std::string _odom_frame, _base_link_frame; 
     static tf::TransformBroadcaster odom_broadcaster;
    tf::Quaternion qt = tf::Quaternion(odomsg->pose.pose.orientation.x,
            odomsg->pose.pose.orientation.y,
            odomsg->pose.pose.orientation.z,
            odomsg->pose.pose.orientation.w);
    tf::StampedTransform fuckyouu =    
          tf::StampedTransform(
                    tf::Transform(qt,
            tf::Vector3(odomsg->pose.pose.position.x,
                        odomsg->pose.pose.position.y,
                        odomsg->pose.pose.position.z)),
                    ros::Time::now(), _odom_frame, _base_link_frame);


    odom_broadcaster.sendTransform(fuckyouu);
}



std::string _odom_frame, _base_link_frame, _laser_link_frame;

int main(int argc, char** argv){
    extern std::string _odom_frame, _base_link_frame, _laser_link_frame;
    ros::init(argc, argv, "pioneer_tf");
    ros::NodeHandle n = ros::NodeHandle("~");
    
    n.param("odom_frame", _odom_frame, std::string("")) ;
    n.param("base_link_frame", _base_link_frame, std::string("")) ;
    n.param("laser_link_frame", _laser_link_frame, std::string("")) ;
    ros::Rate r(100);

    tf::TransformBroadcaster broadcaster;
  
  //subscribe to pose info
    ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>(_odom_frame, 1000, poseCallback);

    while(n.ok()){
        
  broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0), tf::Vector3(0.20, 0.0, 0.29)),
                ros::Time::now(), _base_link_frame, _laser_link_frame));
    ros::spinOnce();        
    r.sleep();
    }
}
