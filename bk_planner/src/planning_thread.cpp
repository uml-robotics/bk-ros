#include <bk_planner/bk_planner.h>

namespace bk_planner {

void
BKPlanner::runPlanningThread()
{
	long period_ms    = (double)1000 * 0.5; // 1/planner_frequency_;
	long wait_time_ms = (double)1000 * 0.5;
	bool made_one_plan = false;
	ros::Rate r(1.0); // 10 hz
	
	ROS_INFO("[planning] bk_planner planning thread started, period is %ld, wait time %ld", period_ms, wait_time_ms);
	
	boost::shared_ptr<segment_lib::SegmentVisualizer> planner_visualizer_;
	planner_visualizer_ = boost::shared_ptr<segment_lib::SegmentVisualizer>
		(new segment_lib::SegmentVisualizer(std::string("planning_visualization")) );
		
	while(true)
	{
		if( getPlannerState() == IN_RECOVERY )
		{
			ROS_INFO("[planning] In recovery");
			doRecovery();
		}
		else if( getPlannerState() == NEED_RECOVERY )
		{
			ROS_INFO("[planning] Starting recovery");
			escalatePlannerState(IN_RECOVERY);
			startRecovery();
		}
		else if( getPlannerState() == NEED_FULL_REPLAN )
		{
			ROS_INFO("[planning] Doing full replan");
			sendResetSignals();
			
			if( doFullReplan() ) {
				setPlannerState(GOOD);
				made_one_plan = true;
			}else {
				ROS_INFO("[planning] Full replan failed, signaling recovery");
				escalatePlannerState(NEED_RECOVERY);
			}
		}
		else if( getPlannerState() == NEED_PARTIAL_REPLAN )
		{
			ROS_INFO("[planning] Doing partial replan");
			
			if( doPartialReplan() ){
				setPlannerState(GOOD);
				made_one_plan = true;
			}else {
				ROS_INFO("[planning] Partial replan failed, signaling full replan");
				escalatePlannerState(NEED_FULL_REPLAN);
			}
		}
		
		if( getPlannerState() == GOOD && made_one_plan )
		{
			//ROS_INFO("[planning] Planner state good.");
			commitPathSegments();
			setFeederEnabled(true);
		}
	
		// publish visualization
		planner_visualizer_->publishVisualization(planner_path_);
		
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
		r.sleep();
	}
}

void
BKPlanner::startRecovery()
{
	return;
}

void
BKPlanner::doRecovery()
{
	// Hack: recovery doesn't exist yet
	setPlannerState(NEED_FULL_REPLAN);
}

bool
BKPlanner::doFullReplan()
{	
	// Get the robot's current pose
	tf::Stamped<tf::Pose> robot_pose;
	if(!planner_costmap_->getRobotPose(robot_pose)){
		ROS_ERROR("[planning] bk_planner cannot make a plan for you because it could not get the start pose of the robot");
		return false;
	}
	geometry_msgs::PoseStamped start;
	tf::poseStampedTFToMsg(robot_pose, start);
	
	// Get the goal
	geometry_msgs::PoseStamped goal = getLatestGoal();
	
	// Clear current path
	dequeueSegments();
	planner_path_.segs.clear();
	
	// Plan to the goal
	bool success = planPointToPoint(start, goal, planner_path_);
	
	// Reindex the path so that all previous segments are invalid
	segment_lib::reindexPath(planner_path_, last_committed_segnum_ + 5);
	
	return success;
}

bool
BKPlanner::doPartialReplan()
{
// return false; // h4x

	// If there aren't outstanding, uncommitted segments, do a full replan instead.
	if(planner_path_.segs.size() < 1){
		ROS_INFO("[planning] No outstanding segments, doing full replan instead.");
		planner_path_.segs.clear();
		
		if( getFeederDistLeft < 
	}
		
	// Clear all but the first uncommitted segment
	if( planner_path_.segs.size() > 1 ) {
		planner_path_.segs.erase( planner_path_.segs.begin()+1, planner_path_.segs.end() );
	}
	
	// TODO: If we're out of segments, but the feeder still has some distance left, replan from the last pose
	
	if( planner_path_.segs.size() > 1 ){
		ROS_ERROR("DERPDEDERP");};
	
	// Plan from the end of the first uncommitted segment to the goal
  geometry_msgs::PoseStamped start = segment_lib::getEndPose(planner_path_.segs.front());
	geometry_msgs::PoseStamped goal  = getLatestGoal();
	
	// Make the plan and append it
	p_nav::Path splice;
	bool success = planPointToPoint(start, goal, splice);
	
	if( success ){
		planner_path_.segs.insert(planner_path_.segs.begin()+1, splice.segs.begin(), splice.segs.end() );
		
		// Resample the path
		//planner_path_ = segment_lib::smoothPath(planner_path_);
		
		// Reindex the path to be continuous with the segments previously committed
		segment_lib::reindexPath(planner_path_, last_committed_segnum_+1 );
	
		return true;
	}
	return false;
}


// Point-to-point planner
bool
BKPlanner::planPointToPoint(const geometry_msgs::PoseStamped& start,
                                 const geometry_msgs::PoseStamped& goal,
                                 precision_navigation_msgs::Path&  path)
{
	/*if( !start.header.frame_id.compare(goal.header.frame_id) ){
		ROS_ERROR("Start and goal poses are in different frames.  What are you trying to pull here?");
		return false;
	}*/
    
  ros::Time t1 = ros::Time::now();
	bool success = lattice_planner_->makeSegmentPlan(start, goal, path);

	if( success == false ){
		ROS_ERROR("[planning] Planner failed!");
		return false;
	}
	ros::Duration t = ros::Time::now() - t1;
	
	ROS_INFO("[planning] Point-point plan made in %.2f seconds. Plan has %lu segments.", t.toSec(), path.segs.size());
	
	path.header.stamp    = ros::Time::now();
	path.header.frame_id = start.header.frame_id;
	
	return true;
}

void
BKPlanner::commitPathSegments()
{
	boost::recursive_mutex::scoped_lock l(committed_path_mutex_);
	
	// Get the current length of the feeder path
	double dist_left = getFeederDistLeft();
	
	p_nav::PathSegment seg_just_committed;
	// Desired distance to add
	double dist_to_add = 1.0 - dist_left;
	double dist_just_committed;
	
	ROS_INFO("Feeder has %.2f left, committing %.2f", dist_left, dist_to_add);
	
	while( planner_path_.segs.size() > 0 && dist_to_add > 0 )
	{
		seg_just_committed  = commitOneSegment();
		dist_just_committed = segment_lib::linDist(seg_just_committed);
		dist_to_add -= dist_just_committed;
	}
	ROS_INFO("Done committing");
}

// Check if we can commit another segment
bool
BKPlanner::canCommitOneSegment()
{
	return(false);
}

p_nav::PathSegment
BKPlanner::commitOneSegment()
{
	precision_navigation_msgs::Path path_to_commit;
	path_to_commit.header = planner_path_.header;
	
	// Commit this segment and get rid of it
	precision_navigation_msgs::PathSegment seg = planner_path_.segs.front();
	planner_path_.segs.erase(planner_path_.segs.begin());
	
	// Remember our last committed pose
	//last_committed_pose_   = segment_lib::getEndPose(seg);
	last_committed_segnum_ = seg.seg_number;
	
	path_to_commit.segs.push_back(seg);
	enqueueSegments(path_to_commit);
	ROS_INFO("[planning] Planner committed segment %d", last_committed_segnum_);
	return seg;
}

};//namespace