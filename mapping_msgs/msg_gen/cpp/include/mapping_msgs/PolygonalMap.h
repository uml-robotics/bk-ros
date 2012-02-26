/* Auto-generated by genmsg_cpp for file /home/bill/dev/bk-ros/mapping_msgs/msg/PolygonalMap.msg */
#ifndef MAPPING_MSGS_MESSAGE_POLYGONALMAP_H
#define MAPPING_MSGS_MESSAGE_POLYGONALMAP_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "geometry_msgs/Polygon.h"
#include "sensor_msgs/ChannelFloat32.h"

namespace mapping_msgs
{
template <class ContainerAllocator>
struct PolygonalMap_ {
  typedef PolygonalMap_<ContainerAllocator> Type;

  PolygonalMap_()
  : header()
  , polygons()
  , chan()
  {
  }

  PolygonalMap_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , polygons(_alloc)
  , chan(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  _polygons_type;
  std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other >  polygons;

  typedef std::vector< ::sensor_msgs::ChannelFloat32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::ChannelFloat32_<ContainerAllocator> >::other >  _chan_type;
  std::vector< ::sensor_msgs::ChannelFloat32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::ChannelFloat32_<ContainerAllocator> >::other >  chan;


  ROS_DEPRECATED uint32_t get_polygons_size() const { return (uint32_t)polygons.size(); }
  ROS_DEPRECATED void set_polygons_size(uint32_t size) { polygons.resize((size_t)size); }
  ROS_DEPRECATED void get_polygons_vec(std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other > & vec) const { vec = this->polygons; }
  ROS_DEPRECATED void set_polygons_vec(const std::vector< ::geometry_msgs::Polygon_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Polygon_<ContainerAllocator> >::other > & vec) { this->polygons = vec; }
  ROS_DEPRECATED uint32_t get_chan_size() const { return (uint32_t)chan.size(); }
  ROS_DEPRECATED void set_chan_size(uint32_t size) { chan.resize((size_t)size); }
  ROS_DEPRECATED void get_chan_vec(std::vector< ::sensor_msgs::ChannelFloat32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::ChannelFloat32_<ContainerAllocator> >::other > & vec) const { vec = this->chan; }
  ROS_DEPRECATED void set_chan_vec(const std::vector< ::sensor_msgs::ChannelFloat32_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::ChannelFloat32_<ContainerAllocator> >::other > & vec) { this->chan = vec; }
private:
  static const char* __s_getDataType_() { return "mapping_msgs/PolygonalMap"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "3ef041ea9abaaa9dac3e5aec60c68a99"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Header header\n\
geometry_msgs/Polygon[] polygons\n\
sensor_msgs/ChannelFloat32[] chan\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Polygon\n\
#A specification of a polygon where the first and last points are assumed to be connected\n\
geometry_msgs/Point32[] points\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
================================================================================\n\
MSG: sensor_msgs/ChannelFloat32\n\
# This message is used by the PointCloud message to hold optional data\n\
# associated with each point in the cloud. The length of the values\n\
# array should be the same as the length of the points array in the\n\
# PointCloud, and each value should be associated with the corresponding\n\
# point.\n\
\n\
# Channel names in existing practice include:\n\
#   \"u\", \"v\" - row and column (respectively) in the left stereo image.\n\
#              This is opposite to usual conventions but remains for\n\
#              historical reasons. The newer PointCloud2 message has no\n\
#              such problem.\n\
#   \"rgb\" - For point clouds produced by color stereo cameras. uint8\n\
#           (R,G,B) values packed into the least significant 24 bits,\n\
#           in order.\n\
#   \"intensity\" - laser or pixel intensity.\n\
#   \"distance\"\n\
\n\
# The channel name should give semantics of the channel (e.g.\n\
# \"intensity\" instead of \"value\").\n\
string name\n\
\n\
# The values array should be 1-1 with the elements of the associated\n\
# PointCloud.\n\
float32[] values\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, polygons);
    ros::serialization::serialize(stream, chan);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, polygons);
    ros::serialization::deserialize(stream, chan);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(polygons);
    size += ros::serialization::serializationLength(chan);
    return size;
  }

  typedef boost::shared_ptr< ::mapping_msgs::PolygonalMap_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mapping_msgs::PolygonalMap_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PolygonalMap
typedef  ::mapping_msgs::PolygonalMap_<std::allocator<void> > PolygonalMap;

typedef boost::shared_ptr< ::mapping_msgs::PolygonalMap> PolygonalMapPtr;
typedef boost::shared_ptr< ::mapping_msgs::PolygonalMap const> PolygonalMapConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::mapping_msgs::PolygonalMap_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::mapping_msgs::PolygonalMap_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace mapping_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::mapping_msgs::PolygonalMap_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::mapping_msgs::PolygonalMap_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::mapping_msgs::PolygonalMap_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3ef041ea9abaaa9dac3e5aec60c68a99";
  }

  static const char* value(const  ::mapping_msgs::PolygonalMap_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3ef041ea9abaaa9dULL;
  static const uint64_t static_value2 = 0xac3e5aec60c68a99ULL;
};

template<class ContainerAllocator>
struct DataType< ::mapping_msgs::PolygonalMap_<ContainerAllocator> > {
  static const char* value() 
  {
    return "mapping_msgs/PolygonalMap";
  }

  static const char* value(const  ::mapping_msgs::PolygonalMap_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::mapping_msgs::PolygonalMap_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
geometry_msgs/Polygon[] polygons\n\
sensor_msgs/ChannelFloat32[] chan\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Polygon\n\
#A specification of a polygon where the first and last points are assumed to be connected\n\
geometry_msgs/Point32[] points\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
================================================================================\n\
MSG: sensor_msgs/ChannelFloat32\n\
# This message is used by the PointCloud message to hold optional data\n\
# associated with each point in the cloud. The length of the values\n\
# array should be the same as the length of the points array in the\n\
# PointCloud, and each value should be associated with the corresponding\n\
# point.\n\
\n\
# Channel names in existing practice include:\n\
#   \"u\", \"v\" - row and column (respectively) in the left stereo image.\n\
#              This is opposite to usual conventions but remains for\n\
#              historical reasons. The newer PointCloud2 message has no\n\
#              such problem.\n\
#   \"rgb\" - For point clouds produced by color stereo cameras. uint8\n\
#           (R,G,B) values packed into the least significant 24 bits,\n\
#           in order.\n\
#   \"intensity\" - laser or pixel intensity.\n\
#   \"distance\"\n\
\n\
# The channel name should give semantics of the channel (e.g.\n\
# \"intensity\" instead of \"value\").\n\
string name\n\
\n\
# The values array should be 1-1 with the elements of the associated\n\
# PointCloud.\n\
float32[] values\n\
\n\
";
  }

  static const char* value(const  ::mapping_msgs::PolygonalMap_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::mapping_msgs::PolygonalMap_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::mapping_msgs::PolygonalMap_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::mapping_msgs::PolygonalMap_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.polygons);
    stream.next(m.chan);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PolygonalMap_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mapping_msgs::PolygonalMap_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::mapping_msgs::PolygonalMap_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "polygons[]" << std::endl;
    for (size_t i = 0; i < v.polygons.size(); ++i)
    {
      s << indent << "  polygons[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Polygon_<ContainerAllocator> >::stream(s, indent + "    ", v.polygons[i]);
    }
    s << indent << "chan[]" << std::endl;
    for (size_t i = 0; i < v.chan.size(); ++i)
    {
      s << indent << "  chan[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::sensor_msgs::ChannelFloat32_<ContainerAllocator> >::stream(s, indent + "    ", v.chan[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // MAPPING_MSGS_MESSAGE_POLYGONALMAP_H

