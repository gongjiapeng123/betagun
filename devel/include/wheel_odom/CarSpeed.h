// Generated by gencpp from file wheel_odom/CarSpeed.msg
// DO NOT EDIT!


#ifndef WHEEL_ODOM_MESSAGE_CARSPEED_H
#define WHEEL_ODOM_MESSAGE_CARSPEED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace wheel_odom
{
template <class ContainerAllocator>
struct CarSpeed_
{
  typedef CarSpeed_<ContainerAllocator> Type;

  CarSpeed_()
    : header()
    , total_left_count(0)
    , total_right_count(0)
    , left_count(0)
    , right_count(0)
    , left_cmd_speed(0)
    , right_cmd_speed(0)
    , car_delta_x(0.0)
    , car_delta_y(0.0)
    , car_delta_th(0.0)
    , left_speed(0.0)
    , right_speed(0.0)
    , vx(0.0)
    , vy(0.0)
    , vth(0.0)  {
    }
  CarSpeed_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , total_left_count(0)
    , total_right_count(0)
    , left_count(0)
    , right_count(0)
    , left_cmd_speed(0)
    , right_cmd_speed(0)
    , car_delta_x(0.0)
    , car_delta_y(0.0)
    , car_delta_th(0.0)
    , left_speed(0.0)
    , right_speed(0.0)
    , vx(0.0)
    , vy(0.0)
    , vth(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint64_t _total_left_count_type;
  _total_left_count_type total_left_count;

   typedef uint64_t _total_right_count_type;
  _total_right_count_type total_right_count;

   typedef int32_t _left_count_type;
  _left_count_type left_count;

   typedef int32_t _right_count_type;
  _right_count_type right_count;

   typedef int32_t _left_cmd_speed_type;
  _left_cmd_speed_type left_cmd_speed;

   typedef int32_t _right_cmd_speed_type;
  _right_cmd_speed_type right_cmd_speed;

   typedef float _car_delta_x_type;
  _car_delta_x_type car_delta_x;

   typedef float _car_delta_y_type;
  _car_delta_y_type car_delta_y;

   typedef float _car_delta_th_type;
  _car_delta_th_type car_delta_th;

   typedef float _left_speed_type;
  _left_speed_type left_speed;

   typedef float _right_speed_type;
  _right_speed_type right_speed;

   typedef float _vx_type;
  _vx_type vx;

   typedef float _vy_type;
  _vy_type vy;

   typedef float _vth_type;
  _vth_type vth;




  typedef boost::shared_ptr< ::wheel_odom::CarSpeed_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::wheel_odom::CarSpeed_<ContainerAllocator> const> ConstPtr;

}; // struct CarSpeed_

typedef ::wheel_odom::CarSpeed_<std::allocator<void> > CarSpeed;

typedef boost::shared_ptr< ::wheel_odom::CarSpeed > CarSpeedPtr;
typedef boost::shared_ptr< ::wheel_odom::CarSpeed const> CarSpeedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::wheel_odom::CarSpeed_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::wheel_odom::CarSpeed_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace wheel_odom

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'wheel_odom': ['/root/workspace/betagun/src/wheel_odom/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::wheel_odom::CarSpeed_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::wheel_odom::CarSpeed_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wheel_odom::CarSpeed_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::wheel_odom::CarSpeed_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wheel_odom::CarSpeed_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::wheel_odom::CarSpeed_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::wheel_odom::CarSpeed_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2f5e9973b77a5094ea93a46fb1e0579f";
  }

  static const char* value(const ::wheel_odom::CarSpeed_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2f5e9973b77a5094ULL;
  static const uint64_t static_value2 = 0xea93a46fb1e0579fULL;
};

template<class ContainerAllocator>
struct DataType< ::wheel_odom::CarSpeed_<ContainerAllocator> >
{
  static const char* value()
  {
    return "wheel_odom/CarSpeed";
  }

  static const char* value(const ::wheel_odom::CarSpeed_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::wheel_odom::CarSpeed_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
uint64 total_left_count\n\
uint64 total_right_count\n\
int32 left_count\n\
int32 right_count\n\
int32 left_cmd_speed\n\
int32 right_cmd_speed\n\
float32 car_delta_x\n\
float32 car_delta_y\n\
float32 car_delta_th\n\
float32 left_speed\n\
float32 right_speed\n\
float32 vx\n\
float32 vy\n\
float32 vth\n\
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

  static const char* value(const ::wheel_odom::CarSpeed_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::wheel_odom::CarSpeed_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.total_left_count);
      stream.next(m.total_right_count);
      stream.next(m.left_count);
      stream.next(m.right_count);
      stream.next(m.left_cmd_speed);
      stream.next(m.right_cmd_speed);
      stream.next(m.car_delta_x);
      stream.next(m.car_delta_y);
      stream.next(m.car_delta_th);
      stream.next(m.left_speed);
      stream.next(m.right_speed);
      stream.next(m.vx);
      stream.next(m.vy);
      stream.next(m.vth);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CarSpeed_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::wheel_odom::CarSpeed_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::wheel_odom::CarSpeed_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "total_left_count: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.total_left_count);
    s << indent << "total_right_count: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.total_right_count);
    s << indent << "left_count: ";
    Printer<int32_t>::stream(s, indent + "  ", v.left_count);
    s << indent << "right_count: ";
    Printer<int32_t>::stream(s, indent + "  ", v.right_count);
    s << indent << "left_cmd_speed: ";
    Printer<int32_t>::stream(s, indent + "  ", v.left_cmd_speed);
    s << indent << "right_cmd_speed: ";
    Printer<int32_t>::stream(s, indent + "  ", v.right_cmd_speed);
    s << indent << "car_delta_x: ";
    Printer<float>::stream(s, indent + "  ", v.car_delta_x);
    s << indent << "car_delta_y: ";
    Printer<float>::stream(s, indent + "  ", v.car_delta_y);
    s << indent << "car_delta_th: ";
    Printer<float>::stream(s, indent + "  ", v.car_delta_th);
    s << indent << "left_speed: ";
    Printer<float>::stream(s, indent + "  ", v.left_speed);
    s << indent << "right_speed: ";
    Printer<float>::stream(s, indent + "  ", v.right_speed);
    s << indent << "vx: ";
    Printer<float>::stream(s, indent + "  ", v.vx);
    s << indent << "vy: ";
    Printer<float>::stream(s, indent + "  ", v.vy);
    s << indent << "vth: ";
    Printer<float>::stream(s, indent + "  ", v.vth);
  }
};

} // namespace message_operations
} // namespace ros

#endif // WHEEL_ODOM_MESSAGE_CARSPEED_H
