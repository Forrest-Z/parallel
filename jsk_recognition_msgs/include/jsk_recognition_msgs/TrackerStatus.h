// Generated by gencpp from file jsk_recognition_msgs/TrackerStatus.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_TRACKERSTATUS_H
#define JSK_RECOGNITION_MSGS_MESSAGE_TRACKERSTATUS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct TrackerStatus_
{
  typedef TrackerStatus_<ContainerAllocator> Type;

  TrackerStatus_()
    : header()
    , is_tracking(false)  {
    }
  TrackerStatus_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , is_tracking(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _is_tracking_type;
  _is_tracking_type is_tracking;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> const> ConstPtr;

}; // struct TrackerStatus_

typedef ::jsk_recognition_msgs::TrackerStatus_<std::allocator<void> > TrackerStatus;

typedef boost::shared_ptr< ::jsk_recognition_msgs::TrackerStatus > TrackerStatusPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::TrackerStatus const> TrackerStatusConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'pcl_msgs': ['/opt/ros/melodic/share/pcl_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'jsk_footstep_msgs': ['/opt/ros/melodic/share/jsk_footstep_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'jsk_recognition_msgs': ['/media/yarten/DATA/Project/ROS/Parallel/src/jsk_recognition_msgs/msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "78e8f06a6b728df052dd181bfcf6c26e";
  }

  static const char* value(const ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x78e8f06a6b728df0ULL;
  static const uint64_t static_value2 = 0x52dd181bfcf6c26eULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/TrackerStatus";
  }

  static const char* value(const ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
# if true, it means the tracker is trying to track object,\n\
# else, the tracker think scene is stable and no need to track\n\
# to save computational resource.\n\
bool is_tracking\n\
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

  static const char* value(const ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.is_tracking);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrackerStatus_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::TrackerStatus_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "is_tracking: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.is_tracking);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_TRACKERSTATUS_H
