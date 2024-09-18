// Generated by gencpp from file gazebo_msgs/SetLinkState.msg
// DO NOT EDIT!


#ifndef GAZEBO_MSGS_MESSAGE_SETLINKSTATE_H
#define GAZEBO_MSGS_MESSAGE_SETLINKSTATE_H

#include <ros/service_traits.h>


#include <gazebo_msgs/SetLinkStateRequest.h>
#include <gazebo_msgs/SetLinkStateResponse.h>


namespace gazebo_msgs
{

struct SetLinkState
{

typedef SetLinkStateRequest Request;
typedef SetLinkStateResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetLinkState
} // namespace gazebo_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::gazebo_msgs::SetLinkState > {
  static const char* value()
  {
    return "8a5146eb66ae4d26b0860b08f3f271be";
  }

  static const char* value(const ::gazebo_msgs::SetLinkState&) { return value(); }
};

template<>
struct DataType< ::gazebo_msgs::SetLinkState > {
  static const char* value()
  {
    return "gazebo_msgs/SetLinkState";
  }

  static const char* value(const ::gazebo_msgs::SetLinkState&) { return value(); }
};


// service_traits::MD5Sum< ::gazebo_msgs::SetLinkStateRequest> should match
// service_traits::MD5Sum< ::gazebo_msgs::SetLinkState >
template<>
struct MD5Sum< ::gazebo_msgs::SetLinkStateRequest>
{
  static const char* value()
  {
    return MD5Sum< ::gazebo_msgs::SetLinkState >::value();
  }
  static const char* value(const ::gazebo_msgs::SetLinkStateRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::gazebo_msgs::SetLinkStateRequest> should match
// service_traits::DataType< ::gazebo_msgs::SetLinkState >
template<>
struct DataType< ::gazebo_msgs::SetLinkStateRequest>
{
  static const char* value()
  {
    return DataType< ::gazebo_msgs::SetLinkState >::value();
  }
  static const char* value(const ::gazebo_msgs::SetLinkStateRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::gazebo_msgs::SetLinkStateResponse> should match
// service_traits::MD5Sum< ::gazebo_msgs::SetLinkState >
template<>
struct MD5Sum< ::gazebo_msgs::SetLinkStateResponse>
{
  static const char* value()
  {
    return MD5Sum< ::gazebo_msgs::SetLinkState >::value();
  }
  static const char* value(const ::gazebo_msgs::SetLinkStateResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::gazebo_msgs::SetLinkStateResponse> should match
// service_traits::DataType< ::gazebo_msgs::SetLinkState >
template<>
struct DataType< ::gazebo_msgs::SetLinkStateResponse>
{
  static const char* value()
  {
    return DataType< ::gazebo_msgs::SetLinkState >::value();
  }
  static const char* value(const ::gazebo_msgs::SetLinkStateResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // GAZEBO_MSGS_MESSAGE_SETLINKSTATE_H