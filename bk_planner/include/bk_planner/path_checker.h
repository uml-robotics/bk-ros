#ifndef PATH_CHECKER_H_
#define PATH_CHECKER_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <precision_navigation_msgs/Path.h>
#include <precision_navigation_msgs/PathSegment.h>
#include <geometry_msgs/Twist.h>

namespace path_checker{
	
	class PathChecker
	{
		public:
			PathChecker(std::string name, boost::shared_ptr<costmap_2d::Costmap2DROS> costmap);
			~PathChecker() {}

			// Fills in a safe velocity to a path based on nearby obstacles
			void assignPathVelocity(precision_navigation_msgs::Path& path);

			// Fills in a safe velocity to a segment based on nearby obstacles
			void assignSegVelocity(precision_navigation_msgs::PathSegment& seg);

			// Returns true if nothing is blocking the segment
			bool isSegClear(const precision_navigation_msgs::PathSegment& seg);

			// Checks for obstacles along the path.  Returns a list of all colliding segment indices
			std::vector<int> getBlockedSegs(precision_navigation_msgs::Path& path);
			
			// Returns the closest obstacle to the robot as its footprint moves along the segment
			double getClosestDist(const precision_navigation_msgs::PathSegment& seg);
			
			// Does a quick check, makes sure the path has all nonzero velocities
			bool isPathClear(const precision_navigation_msgs::Path path);
			
			// Returns the lowest allowed velocity contained in the path
			// Equivalent to min{ all_segs.max_vel }
			double getMinVelocity(const precision_navigation_msgs::Path path);
			
		private:
			boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_;
			ros::NodeHandle           private_nh_;

			// Absolute maximum speed/accel
			geometry_msgs::Twist max_speed_;
			geometry_msgs::Twist max_accel_;

	};//class

};//namespace

#endif
