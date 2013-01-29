/*********************************************************************************/
/* follow.cpp                                                                    */
/* by Mike Lunderville                                                           */
/*                                                                               */
/* Requires move_base and people_tracker. Outputs goals based on tracked target. */
/*********************************************************************************/

// ROS include
#include "ros/ros.h"

// Other ROS includes
#include "tf/transform_listener.h"

// ROS message includes
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "people_msgs/PositionMeasurement.h"

#include <boost/bind.hpp>
#include <cmath>
#include <math.h>

// ROS params
std::string base_link_frame_ = "/base_link";
std::string fixed_frame_ = "/odom";

class FollowMe
{
public:
    // Class variables
    ros::NodeHandle             nh_;    
    tf::TransformListener       tfl_;

    geometry_msgs::PoseStamped  *current_goal;

    double following_dist; // Distance to follow in meters.
    double min_goal_diff;  // Distance that a new goal has to be different by in order to replace current goal.

    // Publishers and subscribers
    ros::Publisher   goal_pub_;
    ros::Subscriber  tracker_sub_;

    // Constructor
    FollowMe(ros::NodeHandle nh) : 
        nh_(nh),
        following_dist(0.3),
        min_goal_diff(0.03)
    {
        current_goal = NULL;
        goal_pub_    = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        tracker_sub_ = nh_.subscribe<people_msgs::PositionMeasurement>("/people_tracker_filter", 10, boost::bind(&FollowMe::trackerCallback, this, _1) );
    }

    // Destructor
    ~FollowMe(){}

    // Callback
    void trackerCallback(const people_msgs::PositionMeasurementConstPtr& position)
    {
        // Gather information from the input data and transform it to the base_link frame
        geometry_msgs::PointStamped goal;
        goal.header = position->header;
        goal.point  = position->pos;

        geometry_msgs::PointStamped base_link_frame_goal;
        try{
            this->tfl_.transformPoint(base_link_frame_, goal, base_link_frame_goal);
        }
        catch (tf::TransformException& ex){
            ROS_ERROR("(follow) Transform in callback failed.");
            return;
        }
        
        // Calculate the new goal
        double dist = sqrt( pow(base_link_frame_goal.point.x,2) + pow(base_link_frame_goal.point.y,2) );

        // If the distance from the robot to the target is too small, just return.        
        if (dist < this->following_dist){
            return;
        }

        // Scale the transformed_pt appropriately, and make it the new goal.
        geometry_msgs::PoseStamped new_goal;
        new_goal.header = base_link_frame_goal.header;
        new_goal.pose.position.x = ((dist - this->following_dist)/dist) * base_link_frame_goal.point.x;
        new_goal.pose.position.y = ((dist - this->following_dist)/dist) * base_link_frame_goal.point.y;
        new_goal.pose.position.z = 0.0;

        // Calculate the orientation of the new_goal in relation to base_link
        double angle = atan2( new_goal.pose.position.y, new_goal.pose.position.x );
        // Convert this angle to a quaternion and put it into new_goal.
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(angle); 
        new_goal.pose.orientation = quat;

        // Transform this new goal to the fixed frame
        geometry_msgs::PoseStamped* fixed_frame_goal = new geometry_msgs::PoseStamped;
        try{
            this->tfl_.transformPose(fixed_frame_, new_goal, *fixed_frame_goal);
        }
        catch (tf::TransformException& ex){
            ROS_ERROR("(follow) Transform (second) in callback failed.");
            return;
        }

        // If the current goal is not NULL (which it would be until the first callback), make sure that the fixed_frame_goal and current_goal are not too similar.
        if (this->current_goal != NULL){
            // Calculate the distance between the two poses.
            dist = sqrt( pow( (this->current_goal->pose.position.x - fixed_frame_goal->pose.position.x), 2) + pow( (this->current_goal->pose.position.y - fixed_frame_goal->pose.position.y), 2) );
            // If the two poses are too close, return and don't replace the old goal or send it to move_base
            if (dist < this->min_goal_diff)
                return;
        }

        // Update the current_goal to the fixed_frame_goal
        delete this->current_goal;
        this->current_goal = fixed_frame_goal;

        // Finally, publish this new pose to move_base.
        this->goal_pub_.publish(*fixed_frame_goal);

        return;
    }

};


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "follow_me");

    ros::NodeHandle pnh("~");
    pnh.getParam("base_link_frame", base_link_frame_);
    pnh.getParam("odom_frame", fixed_frame_);


    ros::NodeHandle nh;
    FollowMe follow(nh);
    ros::spin();

    return 0;
}

