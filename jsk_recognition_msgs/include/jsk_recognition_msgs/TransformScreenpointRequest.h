// Generated by gencpp from file jsk_recognition_msgs/TransformScreenpointRequest.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_TRANSFORMSCREENPOINTREQUEST_H
#define JSK_RECOGNITION_MSGS_MESSAGE_TRANSFORMSCREENPOINTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace jsk_recognition_msgs
{
template <class ContainerAllocator>
struct TransformScreenpointRequest_
{
  typedef TransformScreenpointRequest_<ContainerAllocator> Type;

  TransformScreenpointRequest_()
    : x(0.0)
    , y(0.0)  {
    }
  TransformScreenpointRequest_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)  {
  (void)_alloc;
    }



   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;





  typedef boost::shared_ptr< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> const> ConstPtr;

}; // struct TransformScreenpointRequest_

typedef ::jsk_recognition_msgs::TransformScreenpointRequest_<std::allocator<void> > TransformScreenpointRequest;

typedef boost::shared_ptr< ::jsk_recognition_msgs::TransformScreenpointRequest > TransformScreenpointRequestPtr;
typedef boost::shared_ptr< ::jsk_recognition_msgs::TransformScreenpointRequest const> TransformScreenpointRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jsk_recognition_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'pcl_msgs': ['/opt/ros/melodic/share/pcl_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/melodic/share/sensor_msgs/cmake/../msg'], 'jsk_footstep_msgs': ['/opt/ros/melodic/share/jsk_footstep_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg'], 'jsk_recognition_msgs': ['/media/yarten/DATA/Project/ROS/Parallel/src/jsk_recognition_msgs/msg'], 'actionlib_msgs': ['/opt/ros/melodic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ff8d7d66dd3e4b731ef14a45d38888b6";
  }

  static const char* value(const ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xff8d7d66dd3e4b73ULL;
  static const uint64_t static_value2 = 0x1ef14a45d38888b6ULL;
};

template<class ContainerAllocator>
struct DataType< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jsk_recognition_msgs/TransformScreenpointRequest";
  }

  static const char* value(const ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
float32 x\n\
float32 y\n\
";
  }

  static const char* value(const ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TransformScreenpointRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jsk_recognition_msgs::TransformScreenpointRequest_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_TRANSFORMSCREENPOINTREQUEST_H
