// Generated by gencpp from file autoware_msgs/icp_stat.msg
// DO NOT EDIT!


#ifndef AUTOWARE_MSGS_MESSAGE_ICP_STAT_H
#define AUTOWARE_MSGS_MESSAGE_ICP_STAT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace autoware_msgs
{
template <class ContainerAllocator>
struct icp_stat_
{
  typedef icp_stat_<ContainerAllocator> Type;

  icp_stat_()
    : header()
    , exe_time(0.0)
    , score(0.0)
    , velocity(0.0)
    , acceleration(0.0)
    , use_predict_pose(0)  {
    }
  icp_stat_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , exe_time(0.0)
    , score(0.0)
    , velocity(0.0)
    , acceleration(0.0)
    , use_predict_pose(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _exe_time_type;
  _exe_time_type exe_time;

   typedef float _score_type;
  _score_type score;

   typedef float _velocity_type;
  _velocity_type velocity;

   typedef float _acceleration_type;
  _acceleration_type acceleration;

   typedef int32_t _use_predict_pose_type;
  _use_predict_pose_type use_predict_pose;





  typedef boost::shared_ptr< ::autoware_msgs::icp_stat_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_msgs::icp_stat_<ContainerAllocator> const> ConstPtr;

}; // struct icp_stat_

typedef ::autoware_msgs::icp_stat_<std::allocator<void> > icp_stat;

typedef boost::shared_ptr< ::autoware_msgs::icp_stat > icp_statPtr;
typedef boost::shared_ptr< ::autoware_msgs::icp_stat const> icp_statConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_msgs::icp_stat_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_msgs::icp_stat_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace autoware_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'pcl_msgs': ['/opt/ros/melodic/share/pcl_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'jsk_footstep_msgs': ['/opt/ros/melodic/share/jsk_footstep_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'jsk_recognition_msgs': ['/media/yarten/DATA/Project/ROS/Parallel/src/jsk_recognition_msgs/msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'autoware_msgs': ['/media/yarten/DATA/Project/ROS/Parallel/src/autoware_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::icp_stat_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::icp_stat_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::icp_stat_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::icp_stat_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::icp_stat_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::icp_stat_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_msgs::icp_stat_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b2199054c47ce764e0bad5649c1d2203";
  }

  static const char* value(const ::autoware_msgs::icp_stat_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb2199054c47ce764ULL;
  static const uint64_t static_value2 = 0xe0bad5649c1d2203ULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_msgs::icp_stat_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_msgs/icp_stat";
  }

  static const char* value(const ::autoware_msgs::icp_stat_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_msgs::icp_stat_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float32 exe_time\n\
#int32 iteration\n\
float32 score\n\
float32 velocity\n\
float32 acceleration\n\
int32 use_predict_pose\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::autoware_msgs::icp_stat_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_msgs::icp_stat_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.exe_time);
      stream.next(m.score);
      stream.next(m.velocity);
      stream.next(m.acceleration);
      stream.next(m.use_predict_pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct icp_stat_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autoware_msgs::icp_stat_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_msgs::icp_stat_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "exe_time: ";
    Printer<float>::stream(s, indent + "  ", v.exe_time);
    s << indent << "score: ";
    Printer<float>::stream(s, indent + "  ", v.score);
    s << indent << "velocity: ";
    Printer<float>::stream(s, indent + "  ", v.velocity);
    s << indent << "acceleration: ";
    Printer<float>::stream(s, indent + "  ", v.acceleration);
    s << indent << "use_predict_pose: ";
    Printer<int32_t>::stream(s, indent + "  ", v.use_predict_pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_MSGS_MESSAGE_ICP_STAT_H
