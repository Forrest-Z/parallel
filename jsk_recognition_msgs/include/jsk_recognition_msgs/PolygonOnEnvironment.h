// Generated by gencpp from file jsk_recognition_msgs/PolygonOnEnvironment.msg
// DO NOT EDIT!


#ifndef JSK_RECOGNITION_MSGS_MESSAGE_POLYGONONENVIRONMENT_H
#define JSK_RECOGNITION_MSGS_MESSAGE_POLYGONONENVIRONMENT_H

#include <ros/service_traits.h>


#include <jsk_recognition_msgs/PolygonOnEnvironmentRequest.h>
#include <jsk_recognition_msgs/PolygonOnEnvironmentResponse.h>


namespace jsk_recognition_msgs
{

struct PolygonOnEnvironment
{

typedef PolygonOnEnvironmentRequest Request;
typedef PolygonOnEnvironmentResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct PolygonOnEnvironment
} // namespace jsk_recognition_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::jsk_recognition_msgs::PolygonOnEnvironment > {
  static const char* value()
  {
    return "a8ff091210c071091863d880820e505c";
  }

  static const char* value(const ::jsk_recognition_msgs::PolygonOnEnvironment&) { return value(); }
};

template<>
struct DataType< ::jsk_recognition_msgs::PolygonOnEnvironment > {
  static const char* value()
  {
    return "jsk_recognition_msgs/PolygonOnEnvironment";
  }

  static const char* value(const ::jsk_recognition_msgs::PolygonOnEnvironment&) { return value(); }
};


// service_traits::MD5Sum< ::jsk_recognition_msgs::PolygonOnEnvironmentRequest> should match 
// service_traits::MD5Sum< ::jsk_recognition_msgs::PolygonOnEnvironment > 
template<>
struct MD5Sum< ::jsk_recognition_msgs::PolygonOnEnvironmentRequest>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_recognition_msgs::PolygonOnEnvironment >::value();
  }
  static const char* value(const ::jsk_recognition_msgs::PolygonOnEnvironmentRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_recognition_msgs::PolygonOnEnvironmentRequest> should match 
// service_traits::DataType< ::jsk_recognition_msgs::PolygonOnEnvironment > 
template<>
struct DataType< ::jsk_recognition_msgs::PolygonOnEnvironmentRequest>
{
  static const char* value()
  {
    return DataType< ::jsk_recognition_msgs::PolygonOnEnvironment >::value();
  }
  static const char* value(const ::jsk_recognition_msgs::PolygonOnEnvironmentRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::jsk_recognition_msgs::PolygonOnEnvironmentResponse> should match 
// service_traits::MD5Sum< ::jsk_recognition_msgs::PolygonOnEnvironment > 
template<>
struct MD5Sum< ::jsk_recognition_msgs::PolygonOnEnvironmentResponse>
{
  static const char* value()
  {
    return MD5Sum< ::jsk_recognition_msgs::PolygonOnEnvironment >::value();
  }
  static const char* value(const ::jsk_recognition_msgs::PolygonOnEnvironmentResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::jsk_recognition_msgs::PolygonOnEnvironmentResponse> should match 
// service_traits::DataType< ::jsk_recognition_msgs::PolygonOnEnvironment > 
template<>
struct DataType< ::jsk_recognition_msgs::PolygonOnEnvironmentResponse>
{
  static const char* value()
  {
    return DataType< ::jsk_recognition_msgs::PolygonOnEnvironment >::value();
  }
  static const char* value(const ::jsk_recognition_msgs::PolygonOnEnvironmentResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // JSK_RECOGNITION_MSGS_MESSAGE_POLYGONONENVIRONMENT_H
