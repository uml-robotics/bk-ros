#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_decimator/PCLDecimatorConfig.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "pcl/io/pcd_io.h"
#include "pcl/filters/passthrough.h"
#include <string>
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>

ros::Publisher cloudout_pub_, cloudout_old_pub_;

std::string fieldName_ = "z";
float bandWidth_ = 0.01;
int numBands_ = 21;
float startPoint_ = .4;
float endPoint_ = 1.0;

tf::TransformListener* tfl;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

// CALLBACKS
void pointcloudCallback(const PointCloudXYZ::ConstPtr& cloud_raw)
{
  
  // This is a dumb convoluted process
  // PointCloudXYZ -> PointCloud2 -> PointCloud -> (transform)
  // -> PointCloud -> PointCloud2 -> PointCloudXYZ
  PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
  {
    sensor_msgs::PointCloud2 ros_cloud2, ros_cloud2_tf;
    sensor_msgs::PointCloud  ros_cloud , ros_cloud_tf;

    pcl::toROSMsg(*cloud_raw, ros_cloud2);
    sensor_msgs::convertPointCloud2ToPointCloud(ros_cloud2, ros_cloud);

    try{
      tfl->transformPointCloud("base_link", ros_cloud, ros_cloud_tf);
    } catch( tf::TransformException e ){
      ROS_WARN("Could not transform point cloud.");
      return;
    }
    //ROS_INFO("Transform done");
    sensor_msgs::convertPointCloudToPointCloud2(ros_cloud_tf, ros_cloud2_tf);
    //ROS_INFO("Turning into PCL cloud");
    pcl::fromROSMsg(ros_cloud2_tf, *cloud);
    ROS_INFO("Done transforming.");
  }

  PointCloudXYZ::Ptr cloud_filtered (new PointCloudXYZ);
  PointCloudXYZ::Ptr output_cloud (new PointCloudXYZ);

   // Create the filtering objects
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (fieldName_);
  if(numBands_ <= 1){ 
    // Just center it to be nice
    pass.setFilterLimits ((startPoint_+endPoint_)/2.0, (startPoint_+endPoint_)/2.0+bandWidth_);
    pass.filter(*cloud_filtered);
    *output_cloud = *cloud_filtered;
  } else {
    // Do the first one manually so that certain parameters like frame_id always match
    pass.setFilterLimits (startPoint_, startPoint_+bandWidth_);
    pass.filter(*cloud_filtered);
    *output_cloud = *cloud_filtered;
    float dL = endPoint_-startPoint_;
    
    for(int i = 1; i < numBands_; i++){
      float sLine = startPoint_ + i*dL/(float)(numBands_-1);
      float eLine = sLine + bandWidth_;
      pass.setFilterLimits (sLine, eLine);
      pass.filter(*cloud_filtered);
      *output_cloud += *cloud_filtered;
    }
  }
  ROS_INFO("Publishing");
  cloudout_pub_.publish(*output_cloud);

  // Now, convert to a ROS message and publish it again
  sensor_msgs::PointCloud2 ros_cloud2;
  sensor_msgs::PointCloud  ros_cloud;
  pcl::toROSMsg(*output_cloud, ros_cloud2);
  sensor_msgs::convertPointCloud2ToPointCloud(ros_cloud2, ros_cloud);
  cloudout_old_pub_.publish(ros_cloud);

}



void callback(pcl_decimator::PCLDecimatorConfig &config, uint32_t level)
{
  fieldName_  = config.field_name;
  numBands_   = config.num_slices;
  bandWidth_  = config.slice_width;
  startPoint_ = config.start_threshold;
  endPoint_   = config.end_threshold;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_decimator");
  ros::NodeHandle n;
  tfl = new tf::TransformListener();

  cloudout_pub_     = n.advertise< PointCloudXYZ >("/camera/depth/points_sliced", 1);
  cloudout_old_pub_ = n.advertise< sensor_msgs::PointCloud >("/camera/depth/points_sliced_old", 1);

  ros::Subscriber kinect_sub = n.subscribe("/camera/depth/points", 1, pointcloudCallback);
  
  dynamic_reconfigure::Server<pcl_decimator::PCLDecimatorConfig> srv;
  dynamic_reconfigure::Server<pcl_decimator::PCLDecimatorConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  srv.setCallback(f);

  ros::spin();

  delete tfl;
  return 0;
}




