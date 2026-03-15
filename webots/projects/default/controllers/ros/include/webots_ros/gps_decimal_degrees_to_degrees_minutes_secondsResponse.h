/*
 * Copyright 1996-2022 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef WEBOTS_ROS_MESSAGE_GPS_DECIMAL_DEGREES_TO_DEGREES_MINUTES_SECONDSRESPONSE_H
#define WEBOTS_ROS_MESSAGE_GPS_DECIMAL_DEGREES_TO_DEGREES_MINUTES_SECONDSRESPONSE_H

#include <string>
#include <vector>
#include <map>

#include "ros/types.h"
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"



namespace webots_ros
{
template <class ContainerAllocator>
struct gps_decimal_degrees_to_degrees_minutes_secondsResponse_
{
  typedef gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> Type;

  gps_decimal_degrees_to_degrees_minutes_secondsResponse_()
    : degreesMinutesSeconds()  {
    }
  gps_decimal_degrees_to_degrees_minutes_secondsResponse_(const ContainerAllocator& _alloc)
    : degreesMinutesSeconds(_alloc)  {
    }

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >   _degreesMinutesSeconds_type;
  _degreesMinutesSeconds_type degreesMinutesSeconds;



  typedef boost::shared_ptr< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<std::allocator<void> > gps_decimal_degrees_to_degrees_minutes_secondsResponse;

typedef boost::shared_ptr< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse > gps_decimal_degrees_to_degrees_minutes_secondsResponsePtr;
typedef boost::shared_ptr< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse const> gps_decimal_degrees_to_degrees_minutes_secondsResponseConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace webots_ros

namespace ros
{
namespace message_traits
{

// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/groovy/share/std_msgs/msg'], 'webots_ros': ['/home/simon/my_Webots_Projects/controllers/ros_controller/catkin_ws/src/webots_ros/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2911ee9051e401397c9b1e29a01f7ead";
  }

  static const char* value(const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0147e4f36cba5cdaULL;
  static const uint64_t static_value2 = 0x7fa39c089e493413ULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/gps_decimal_degrees_to_degrees_minutes_secondsResponse";
  }

  static const char* value(const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string degreesMinutesSeconds\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.degreesMinutesSeconds);

    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  };

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::gps_decimal_degrees_to_degrees_minutes_secondsResponse_<ContainerAllocator>& v)
  {
        s << indent << "degreesMinutesSeconds: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.degreesMinutesSeconds);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_GPS_DECIMAL_DEGREES_TO_DEGREES_MINUTES_SECONDSRESPONSE_H
