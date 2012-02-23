#include <bk_planner/bk_planner.h>

namespace bk_planner {

BKPlanner::BKPlanner(std::string name, tf::TransformListener& tf):
	nh_      (),
	priv_nh_ ("~"),
	tf_      (tf)
{
	// This node subscribes to a goal pose.
	goal_sub_ = nh_.subscribe("goal", 1, &BKPlanner::goalCB, this);
	

	
	// Initialize state variables
	got_new_goal_          = false;
	
	planner_costmap_ = boost::shared_ptr<costmap_2d::Costmap2DROS>
		(new costmap_2d::Costmap2DROS("local_costmap", tf_) );	          
	path_checker_    = boost::shared_ptr<path_checker::PathChecker>
		(new path_checker::PathChecker("path_checker", planner_costmap_));
	
	// Get the robot's current pose and set a goal there
	tf::Stamped<tf::Pose> robot_pose;
	planner_costmap_->getRobotPose(robot_pose);
	PoseStamped start_pose;
	tf::poseStampedTFToMsg(robot_pose, start_pose);
	start_pose = poseToGlobalFrame(start_pose);
	setNewGoal(start_pose);
	
	// Create the planner and feeder threads
	planner_ = boost::shared_ptr<BKPlanningThread>
		(new BKPlanningThread(this));		
	feeder_   = boost::shared_ptr<BKFeederThread>
		(new BKFeederThread(this));
		
	// Kick off the threads
	planning_thread_ = boost::shared_ptr<boost::thread>
		(new boost::thread(boost::bind(&BKPlanningThread::run, planner_)) );
	feeder_thread_   = boost::shared_ptr<boost::thread>
		(new boost::thread(boost::bind(&BKFeederThread::run  , feeder_ )) );

	ROS_INFO("BKPlanner constructor finished");
	return;
}

BKPlanner::~BKPlanner()
{
	terminateThreads();
}

void
BKPlanner::terminateThreads()
{
	planning_thread_->interrupt();
	planning_thread_->join();

	feeder_thread_->interrupt();
	feeder_thread_->join();
}

void
BKPlanner::goalCB(const PoseStamped::ConstPtr& goal_ptr)
{
	PoseStamped goal = *goal_ptr;

	ROS_INFO("[Goal callback] Got new goal: (%.2f,%.2f) in frame %s", goal.pose.position.x, goal.pose.position.y, goal.header.frame_id.c_str());
	
	setNewGoal(poseToGlobalFrame(goal));
	planner_->escalatePlannerState(NEED_PARTIAL_REPLAN);
}

PoseStamped
BKPlanner::poseToGlobalFrame(const PoseStamped& pose_msg)
{
	std::string global_frame = planner_costmap_->getGlobalFrameID();
	tf::Stamped<tf::Pose> goal_pose, global_pose;
	poseStampedMsgToTF(pose_msg, goal_pose);

	//just get the latest available transform
	goal_pose.stamp_ = ros::Time();

	try {
		tf_.transformPose(global_frame, goal_pose, global_pose);
	}
	catch(tf::TransformException& ex) {
		ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
		goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
		return pose_msg;
	}

	PoseStamped global_pose_msg;
	tf::poseStampedTFToMsg(global_pose, global_pose_msg);
	return global_pose_msg;
}

void 
BKPlanner::setNewGoal(PoseStamped new_goal)
{
	boost::recursive_mutex::scoped_try_lock l(goal_mutex_);
	while(!l){
		l = boost::recursive_mutex::scoped_try_lock(goal_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	got_new_goal_ = true;
	latest_goal_  = new_goal;
}

bool 
BKPlanner::gotNewGoal()
{
	boost::recursive_mutex::scoped_try_lock l(goal_mutex_);
	while(!l){
		l = boost::recursive_mutex::scoped_try_lock(goal_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	
	return got_new_goal_;
}

PoseStamped 
BKPlanner::getLatestGoal()
{
	boost::recursive_mutex::scoped_try_lock l(goal_mutex_);
	while(!l){
		l = boost::recursive_mutex::scoped_try_lock(goal_mutex_);
		boost::this_thread::sleep(boost::posix_time::milliseconds(10));
	}
	got_new_goal_ = false;
	return latest_goal_;
}


};// namespace

int
main(int argc, char** argv)
{
	ros::init(argc, argv, "bk_planner_node");
	tf::TransformListener tf(ros::Duration(10));

	try	{
		bk_planner::BKPlanner bkp("bk_planner", tf);

		// All callbacks are taken care of in the main thread
		while(bkp.nh_.ok()) {
			ros::spinOnce();
		}
	}
	// The planner will go out of scope here and call its destructor.
	// It throws a lock error, possibly because terminate() is called on the threads.
	catch(boost::lock_error e) {
		cout << "Boost threw a lock error but Honey Badger don't care, Honey Badger don't give a &$%#\n";
	}
	
  return(0);
}
