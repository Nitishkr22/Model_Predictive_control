// Generated by gencpp from file genesis_msgs/WheelSpeedReport.msg
// DO NOT EDIT!


#ifndef GENESIS_MSGS_MESSAGE_WHEELSPEEDREPORT_H
#define GENESIS_MSGS_MESSAGE_WHEELSPEEDREPORT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace genesis_msgs
{
template <class ContainerAllocator>
struct WheelSpeedReport_
{
  typedef WheelSpeedReport_<ContainerAllocator> Type;

  WheelSpeedReport_()
    : header()
    , wheel_speed_fl(0.0)
    , wheel_speed_fr(0.0)
    , wheel_speed_rl(0.0)
    , wheel_speed_rr(0.0)  {
    }
  WheelSpeedReport_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , wheel_speed_fl(0.0)
    , wheel_speed_fr(0.0)
    , wheel_speed_rl(0.0)
    , wheel_speed_rr(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _wheel_speed_fl_type;
  _wheel_speed_fl_type wheel_speed_fl;

   typedef float _wheel_speed_fr_type;
  _wheel_speed_fr_type wheel_speed_fr;

   typedef float _wheel_speed_rl_type;
  _wheel_speed_rl_type wheel_speed_rl;

   typedef float _wheel_speed_rr_type;
  _wheel_speed_rr_type wheel_speed_rr;





  typedef boost::shared_ptr< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> const> ConstPtr;

}; // struct WheelSpeedReport_

typedef ::genesis_msgs::WheelSpeedReport_<std::allocator<void> > WheelSpeedReport;

typedef boost::shared_ptr< ::genesis_msgs::WheelSpeedReport > WheelSpeedReportPtr;
typedef boost::shared_ptr< ::genesis_msgs::WheelSpeedReport const> WheelSpeedReportConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::genesis_msgs::WheelSpeedReport_<ContainerAllocator1> & lhs, const ::genesis_msgs::WheelSpeedReport_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.wheel_speed_fl == rhs.wheel_speed_fl &&
    lhs.wheel_speed_fr == rhs.wheel_speed_fr &&
    lhs.wheel_speed_rl == rhs.wheel_speed_rl &&
    lhs.wheel_speed_rr == rhs.wheel_speed_rr;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::genesis_msgs::WheelSpeedReport_<ContainerAllocator1> & lhs, const ::genesis_msgs::WheelSpeedReport_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace genesis_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a53e80a356e4025a8aac3ddad6c964fd";
  }

  static const char* value(const ::genesis_msgs::WheelSpeedReport_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa53e80a356e4025aULL;
  static const uint64_t static_value2 = 0x8aac3ddad6c964fdULL;
};

template<class ContainerAllocator>
struct DataType< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> >
{
  static const char* value()
  {
    return "genesis_msgs/WheelSpeedReport";
  }

  static const char* value(const ::genesis_msgs::WheelSpeedReport_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"# Wheel Speeds\n"
"float32 wheel_speed_fl # kph\n"
"float32 wheel_speed_fr # kph\n"
"float32 wheel_speed_rl # kph\n"
"float32 wheel_speed_rr # kph\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
;
  }

  static const char* value(const ::genesis_msgs::WheelSpeedReport_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.wheel_speed_fl);
      stream.next(m.wheel_speed_fr);
      stream.next(m.wheel_speed_rl);
      stream.next(m.wheel_speed_rr);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WheelSpeedReport_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::genesis_msgs::WheelSpeedReport_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::genesis_msgs::WheelSpeedReport_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "wheel_speed_fl: ";
    Printer<float>::stream(s, indent + "  ", v.wheel_speed_fl);
    s << indent << "wheel_speed_fr: ";
    Printer<float>::stream(s, indent + "  ", v.wheel_speed_fr);
    s << indent << "wheel_speed_rl: ";
    Printer<float>::stream(s, indent + "  ", v.wheel_speed_rl);
    s << indent << "wheel_speed_rr: ";
    Printer<float>::stream(s, indent + "  ", v.wheel_speed_rr);
  }
};

} // namespace message_operations
} // namespace ros

#endif // GENESIS_MSGS_MESSAGE_WHEELSPEEDREPORT_H
