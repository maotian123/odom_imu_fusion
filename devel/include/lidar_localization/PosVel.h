// Generated by gencpp from file lidar_localization/PosVel.msg
// DO NOT EDIT!


#ifndef LIDAR_LOCALIZATION_MESSAGE_POSVEL_H
#define LIDAR_LOCALIZATION_MESSAGE_POSVEL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

namespace lidar_localization
{
template <class ContainerAllocator>
struct PosVel_
{
  typedef PosVel_<ContainerAllocator> Type;

  PosVel_()
    : header()
    , child_frame_id()
    , position()
    , velocity()  {
    }
  PosVel_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , child_frame_id(_alloc)
    , position(_alloc)
    , velocity(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _child_frame_id_type;
  _child_frame_id_type child_frame_id;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;





  typedef boost::shared_ptr< ::lidar_localization::PosVel_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::lidar_localization::PosVel_<ContainerAllocator> const> ConstPtr;

}; // struct PosVel_

typedef ::lidar_localization::PosVel_<std::allocator<void> > PosVel;

typedef boost::shared_ptr< ::lidar_localization::PosVel > PosVelPtr;
typedef boost::shared_ptr< ::lidar_localization::PosVel const> PosVelConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::lidar_localization::PosVel_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::lidar_localization::PosVel_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::lidar_localization::PosVel_<ContainerAllocator1> & lhs, const ::lidar_localization::PosVel_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.child_frame_id == rhs.child_frame_id &&
    lhs.position == rhs.position &&
    lhs.velocity == rhs.velocity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::lidar_localization::PosVel_<ContainerAllocator1> & lhs, const ::lidar_localization::PosVel_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace lidar_localization

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::lidar_localization::PosVel_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::lidar_localization::PosVel_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_localization::PosVel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::lidar_localization::PosVel_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_localization::PosVel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::lidar_localization::PosVel_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::lidar_localization::PosVel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "48ff785a1cb9d87a1278d0b5440187b5";
  }

  static const char* value(const ::lidar_localization::PosVel_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x48ff785a1cb9d87aULL;
  static const uint64_t static_value2 = 0x1278d0b5440187b5ULL;
};

template<class ContainerAllocator>
struct DataType< ::lidar_localization::PosVel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "lidar_localization/PosVel";
  }

  static const char* value(const ::lidar_localization::PosVel_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::lidar_localization::PosVel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# timestamp of synced GNSS-odo measurement:\n"
"Header header\n"
"\n"
"string child_frame_id\n"
"\n"
"# a. position:\n"
"geometry_msgs/Point position\n"
"\n"
"# b. velocity:\n"
"geometry_msgs/Vector3 velocity\n"
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
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::lidar_localization::PosVel_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::lidar_localization::PosVel_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.child_frame_id);
      stream.next(m.position);
      stream.next(m.velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PosVel_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::lidar_localization::PosVel_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::lidar_localization::PosVel_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "child_frame_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.child_frame_id);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // LIDAR_LOCALIZATION_MESSAGE_POSVEL_H