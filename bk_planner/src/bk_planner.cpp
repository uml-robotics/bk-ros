#include <bk_planner/bk_planner.h>

namespace bk_planner {

BKPlanner::BKPlanner(std::string name, tf::TransformListener& tf):
	nh_           (),
	priv_nh_      ("~"),
	tf_           (tf),
	got_new_goal_ (false)
{
	// Initialize the cost map and path checker
	planner_costmap_ = shared_ptr<costmap_2d::Costmap2DROS>
		(new costmap_2d::Costmap2DROS("local_costmap", tf_) );	          
	path_checker_    = shared_ptr<path_checker::PathChecker>
		(new path_checker::PathChecker("path_checker", planner_costmap_));
	
	priv_nh_.param("goal_hysteresis", goal_hysteresis_, 0.2);
	ROS_INFO("[bk_planner] Goal hysteresis is %.2f", goal_hysteresis_);
	
	// Get the robot's current pose and set a goal there
	tf::Stamped<tf::Pose> robot_pose;
	planner_costmap_->getRobotPose(robot_pose);
	PoseStamped start_pose;
	tf::poseStampedTFToMsg(robot_pose, start_pose);
	bool ret = poseToGlobalFrame(start_pose, start_pose);
	
	if( ret ) {
		setNewGoal(start_pose);
	}
	else {
		ROS_ERROR("[bk_planner] Could not transform start pose");
	}
	
	// Create the planner and feeder threads
	planner_ = shared_ptr<BKPlanningThread>
		(new BKPlanningThread(this) );		
	feeder_   = shared_ptr<BKFeederThread>
		(new BKFeederThread  (this) );
	
	// Set their references to each other
	planner_->setFeeder (feeder_);
	feeder_ ->setPlanner(planner_);
	
	// Kick off the threads
	planning_thread_ = shared_ptr<boost::thread>
		(new boost::thread(boost::bind(&BKPlanningThread::run, planner_)) );
	feeder_thread_   = shared_ptr<boost::thread>
		(new boost::thread(boost::bind(&BKFeederThread::run  , feeder_ )) );

	// This node subscribes to a goal pose.
	goal_sub_ = nh_.subscribe("goal", 1, &BKPlanner::goalCB, this);
	
	// ROS_INFO("BKPlanner constructor finished");
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
	feeder_thread_  ->interrupt();
	planning_thread_->join();
	feeder_thread_  ->join();
}

double dist( const PoseStamped& p1, const PoseStamped& p2 )
{
	return sqrt(
              ((p1.pose.position.x - p2.pose.position.x)
              *(p1.pose.position.x - p2.pose.position.x))+
              ((p1.pose.position.y - p2.pose.position.y)
              *(p1.pose.position.y - p2.pose.position.y))+
              ((p1.pose.position.z - p2.pose.position.z)
              *(p1.pose.position.z - p2.pose.position.z)) );
}

void
BKPlanner::goalCB(const PoseStamped::ConstPtr& goal_ptr)
{
	//PoseStamped curr_goal = getLatestGoal();

	ROS_INFO_THROTTLE(2,"[Goal callback] Got new goal: (%.2f,%.2f) in frame %s", goal_ptr->pose.position.x, goal_ptr->pose.position.y, goal_ptr->header.frame_id.c_str());
	
	PoseStamped new_goal;
	
	/*if( !poseToGlobalFrame(*goal_ptr, new_goal) )
	{
		ROS_ERROR("[Goal callback] Could not transform goal to global frame");
		return;
	}*/
	new_goal = *goal_ptr;
	got_new_goal_ = true;
	
	/*double d = dist(curr_goal, new_goal);
	if( d > goal_hysteresis_ )
	{
		setNewGoal(new_goal);
		got_new_goal_ = true;
	}*/
}

bool
BKPlanner::poseToGlobalFrame(const PoseStamped& pose, PoseStamped& transformed)
{
	std::string global_frame = planner_costmap_->getGlobalFrameID();
	tf::Stamped<tf::Pose> pose_tf, global_tf;
	poseStampedMsgToTF(pose, pose_tf);

	try {
		tf_.transformPose(global_frame, pose_tf, global_tf);
	}
	catch(tf::TransformException& ex) {
		ROS_ERROR("[poseToGlobalFrame] Failed to transform goal pose from \"%s\" to \"%s\" frame: %s",
		pose_tf.frame_id_.c_str(), global_frame.c_str(), ex.what());
		return false;
	}

	tf::poseStampedTFToMsg(global_tf, transformed);
	return true;
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
	
	PoseStamped transformed_goal;
	if( poseToGlobalFrame(latest_goal_, transformed_goal) ) {
		return transformed_goal;
	}
	else {
		ROS_ERROR("[getLatestGoal] Failed to transform!");
		return latest_goal_;
	}
}

};// namespace

int
main(int argc, char** argv)
{
	ros::init(argc, argv, "bk_planner_node");
	ros::NodeHandle nh;
	tf::TransformListener tf(ros::Duration(10));
	
	try	{
		bk_planner::BKPlanner bkp("bk_planner", tf);

		// All callbacks are taken care of in the main thread
		while(nh.ok()) {
			ros::spinOnce();
		}
	}
	// The planner will go out of scope here and call its destructor.
	// It might throw a lock error because of interraction between the threads and mutexes
	catch(boost::lock_error e) {
		cout << "Boost threw a lock error (" << e.what() << ")\n";
		cout << "Honey Badger don't care, Honey Badger don't give a &$%#\n\n";
	}
	
	return(0);
}
