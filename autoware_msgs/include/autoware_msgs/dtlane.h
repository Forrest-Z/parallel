// Generated by gencpp from file autoware_msgs/dtlane.msg
// DO NOT EDIT!


#ifndef AUTOWARE_MSGS_MESSAGE_DTLANE_H
#define AUTOWARE_MSGS_MESSAGE_DTLANE_H


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
struct dtlane_
{
  typedef dtlane_<ContainerAllocator> Type;

  dtlane_()
    : dist(0.0)
    , dir(0.0)
    , apara(0.0)
    , r(0.0)
    , slope(0.0)
    , cant(0.0)
    , lw(0.0)
    , rw(0.0)  {
    }
  dtlane_(const ContainerAllocator& _alloc)
    : dist(0.0)
    , dir(0.0)
    , apara(0.0)
    , r(0.0)
    , slope(0.0)
    , cant(0.0)
    , lw(0.0)
    , rw(0.0)  {
  (void)_alloc;
    }



   typedef double _dist_type;
  _dist_type dist;

   typedef double _dir_type;
  _dir_type dir;

   typedef double _apara_type;
  _apara_type apara;

   typedef double _r_type;
  _r_type r;

   typedef double _slope_type;
  _slope_type slope;

   typedef double _cant_type;
  _cant_type cant;

   typedef double _lw_type;
  _lw_type lw;

   typedef double _rw_type;
  _rw_type rw;





  typedef boost::shared_ptr< ::autoware_msgs::dtlane_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_msgs::dtlane_<ContainerAllocator> const> ConstPtr;

}; // struct dtlane_

typedef ::autoware_msgs::dtlane_<std::allocator<void> > dtlane;

typedef boost::shared_ptr< ::autoware_msgs::dtlane > dtlanePtr;
typedef boost::shared_ptr< ::autoware_msgs::dtlane const> dtlaneConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_msgs::dtlane_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_msgs::dtlane_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::autoware_msgs::dtlane_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::dtlane_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::dtlane_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::dtlane_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::dtlane_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::dtlane_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_msgs::dtlane_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1e60021b42021278e47be71f881d31aa";
  }

  static const char* value(const ::autoware_msgs::dtlane_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1e60021b42021278ULL;
  static const uint64_t static_value2 = 0xe47be71f881d31aaULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_msgs::dtlane_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_msgs/dtlane";
  }

  static const char* value(const ::autoware_msgs::dtlane_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_msgs::dtlane_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 dist\n\
float64 dir\n\
float64 apara\n\
float64 r\n\
float64 slope\n\
float64 cant\n\
float64 lw\n\
float64 rw\n\
";
  }

  static const char* value(const ::autoware_msgs::dtlane_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_msgs::dtlane_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.dist);
      stream.next(m.dir);
      stream.next(m.apara);
      stream.next(m.r);
      stream.next(m.slope);
      stream.next(m.cant);
      stream.next(m.lw);
      stream.next(m.rw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct dtlane_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autoware_msgs::dtlane_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_msgs::dtlane_<ContainerAllocator>& v)
  {
    s << indent << "dist: ";
    Printer<double>::stream(s, indent + "  ", v.dist);
    s << indent << "dir: ";
    Printer<double>::stream(s, indent + "  ", v.dir);
    s << indent << "apara: ";
    Printer<double>::stream(s, indent + "  ", v.apara);
    s << indent << "r: ";
    Printer<double>::stream(s, indent + "  ", v.r);
    s << indent << "slope: ";
    Printer<double>::stream(s, indent + "  ", v.slope);
    s << indent << "cant: ";
    Printer<double>::stream(s, indent + "  ", v.cant);
    s << indent << "lw: ";
    Printer<double>::stream(s, indent + "  ", v.lw);
    s << indent << "rw: ";
    Printer<double>::stream(s, indent + "  ", v.rw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_MSGS_MESSAGE_DTLANE_H
