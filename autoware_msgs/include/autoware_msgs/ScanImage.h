// Generated by gencpp from file autoware_msgs/ScanImage.msg
// DO NOT EDIT!


#ifndef AUTOWARE_MSGS_MESSAGE_SCANIMAGE_H
#define AUTOWARE_MSGS_MESSAGE_SCANIMAGE_H


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
struct ScanImage_
{
  typedef ScanImage_<ContainerAllocator> Type;

  ScanImage_()
    : header()
    , distance()
    , intensity()
    , max_y(0)
    , min_y(0)  {
    }
  ScanImage_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , distance(_alloc)
    , intensity(_alloc)
    , max_y(0)
    , min_y(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _distance_type;
  _distance_type distance;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _intensity_type;
  _intensity_type intensity;

   typedef int32_t _max_y_type;
  _max_y_type max_y;

   typedef int32_t _min_y_type;
  _min_y_type min_y;





  typedef boost::shared_ptr< ::autoware_msgs::ScanImage_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_msgs::ScanImage_<ContainerAllocator> const> ConstPtr;

}; // struct ScanImage_

typedef ::autoware_msgs::ScanImage_<std::allocator<void> > ScanImage;

typedef boost::shared_ptr< ::autoware_msgs::ScanImage > ScanImagePtr;
typedef boost::shared_ptr< ::autoware_msgs::ScanImage const> ScanImageConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_msgs::ScanImage_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_msgs::ScanImage_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace autoware_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'pcl_msgs': ['/opt/ros/melodic/share/pcl_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'jsk_footstep_msgs': ['/opt/ros/melodic/share/jsk_footstep_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'jsk_recognition_msgs': ['/home/yul/Documents/lidar_process/src/jsk/jsk_recognition_msgs/msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'autoware_msgs': ['/home/yul/Documents/lidar_process/src/autoware_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::ScanImage_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::ScanImage_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::ScanImage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::ScanImage_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::ScanImage_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::ScanImage_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_msgs::ScanImage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d3276ec225af95b58af799f4633b1f5d";
  }

  static const char* value(const ::autoware_msgs::ScanImage_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd3276ec225af95b5ULL;
  static const uint64_t static_value2 = 0x8af799f4633b1f5dULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_msgs::ScanImage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_msgs/ScanImage";
  }

  static const char* value(const ::autoware_msgs::ScanImage_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_msgs::ScanImage_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
float32[] distance\n\
float32[] intensity\n\
int32 max_y\n\
int32 min_y\n\
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

  static const char* value(const ::autoware_msgs::ScanImage_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_msgs::ScanImage_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.distance);
      stream.next(m.intensity);
      stream.next(m.max_y);
      stream.next(m.min_y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ScanImage_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autoware_msgs::ScanImage_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_msgs::ScanImage_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "distance[]" << std::endl;
    for (size_t i = 0; i < v.distance.size(); ++i)
    {
      s << indent << "  distance[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.distance[i]);
    }
    s << indent << "intensity[]" << std::endl;
    for (size_t i = 0; i < v.intensity.size(); ++i)
    {
      s << indent << "  intensity[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.intensity[i]);
    }
    s << indent << "max_y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.max_y);
    s << indent << "min_y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.min_y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_MSGS_MESSAGE_SCANIMAGE_H
