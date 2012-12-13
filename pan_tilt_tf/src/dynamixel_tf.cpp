//transform publisher
//Pitch and yaw
//base_link -> pan_link

//pan_link -> camera_link
#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <dynamixel_msgs/JointState.h>

double _p_offset;
void panCallback(const dynamixel_msgs::JointState msg)
{
    extern std::string _namespace;
    static tf::TransformBroadcaster pan_broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(tf::Quaternion(msg.current_pos,0,0));
    pan_broadcaster.sendTransform(tf::StampedTransform( transform, ros::Time::now(), "/pan_link", "/tilt_link"));
/*
        tf::StampedTransform(
            tf::Transform( tf::Quaternion( 1, 1, 1, 1),
            tf::Vector3(0, 0, 0)), msg.header.stamp,"/robot_brain_1/base_link" , "/robot_brain_1/pan_link" ));
*/
}

void tiltCallback(const dynamixel_msgs::JointState msg)
{
    extern std::string _namespace;
    static tf::TransformBroadcaster tilt_broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(tf::Quaternion(0,- msg.current_pos - _p_offset,0));
    tilt_broadcaster.sendTransform(tf::StampedTransform( transform, ros::Time::now(), "/tilt_link", "camera_link"));
/*
        tf::StampedTransform(
            tf::Transform( tf::Quaternion(1, 1, 1, 1),
            tf::Vector3(0, 0, 0)),msg.header.stamp, "/robot_brain_1/pan_link", "/robot_brain_1/tilt_link"));
*/
}

int main(int argc, char** argv) {
    extern std::string _namespace;
    ros::init(argc, argv, "pioneer_tf");
    ros::NodeHandle n = ros::NodeHandle("~");

    n.param("p_offset", _p_offset, 0.0);
    printf("OFFSET IS [%f]\n",_p_offset);
    tf::TransformBroadcaster broadcaster;
//   printf("*****************************************************************\n ZOMG [%s]",_namespace.c_str() ); 
    ros::Subscriber pan_sub = n.subscribe<dynamixel_msgs::JointState>( "/pan_controller/state", 1, panCallback);
    ros::Subscriber tilt_sub = n.subscribe<dynamixel_msgs::JointState>( "/tilt_controller/state", 1, tiltCallback);
    
    while(n.ok())
    {
        ros::spin();
    }
}
