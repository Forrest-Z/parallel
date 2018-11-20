// Generated by gencpp from file autoware_msgs/ConfigRandomFilter.msg
// DO NOT EDIT!


#ifndef AUTOWARE_MSGS_MESSAGE_CONFIGRANDOMFILTER_H
#define AUTOWARE_MSGS_MESSAGE_CONFIGRANDOMFILTER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace autoware_msgs
{
template <class ContainerAllocator>
struct ConfigRandomFilter_
{
  typedef ConfigRandomFilter_<ContainerAllocator> Type;

  ConfigRandomFilter_()
    : sample_num(0)
    , measurement_range(0.0)  {
    }
  ConfigRandomFilter_(const ContainerAllocator& _alloc)
    : sample_num(0)
    , measurement_range(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _sample_num_type;
  _sample_num_type sample_num;

   typedef float _measurement_range_type;
  _measurement_range_type measurement_range;





  typedef boost::shared_ptr< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> const> ConstPtr;

}; // struct ConfigRandomFilter_

typedef ::autoware_msgs::ConfigRandomFilter_<std::allocator<void> > ConfigRandomFilter;

typedef boost::shared_ptr< ::autoware_msgs::ConfigRandomFilter > ConfigRandomFilterPtr;
typedef boost::shared_ptr< ::autoware_msgs::ConfigRandomFilter const> ConfigRandomFilterConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace autoware_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'pcl_msgs': ['/opt/ros/melodic/share/pcl_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'jsk_footstep_msgs': ['/opt/ros/melodic/share/jsk_footstep_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'jsk_recognition_msgs': ['/media/yarten/DATA/Project/ROS/Parallel/src/jsk_recognition_msgs/msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg'], 'autoware_msgs': ['/media/yarten/DATA/Project/ROS/Parallel/src/autoware_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3c62131ed7d7074af43c78ec79a1aa05";
  }

  static const char* value(const ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3c62131ed7d7074aULL;
  static const uint64_t static_value2 = 0xf43c78ec79a1aa05ULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_msgs/ConfigRandomFilter";
  }

  static const char* value(const ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 sample_num\n\
float32 measurement_range\n\
";
  }

  static const char* value(const ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sample_num);
      stream.next(m.measurement_range);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ConfigRandomFilter_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_msgs::ConfigRandomFilter_<ContainerAllocator>& v)
  {
    s << indent << "sample_num: ";
    Printer<int32_t>::stream(s, indent + "  ", v.sample_num);
    s << indent << "measurement_range: ";
    Printer<float>::stream(s, indent + "  ", v.measurement_range);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_MSGS_MESSAGE_CONFIGRANDOMFILTER_H
