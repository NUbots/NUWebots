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

#ifndef WEBOTS_ROS_MESSAGE_MOTOR_SET_CONTROL_PIDREQUEST_H
#define WEBOTS_ROS_MESSAGE_MOTOR_SET_CONTROL_PIDREQUEST_H

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
struct motor_set_control_pidRequest_
{
  typedef motor_set_control_pidRequest_<ContainerAllocator> Type;

  motor_set_control_pidRequest_()
    : controlp(0.0)
    , controli(0.0)
    , controld(0.0)  {
    }
  motor_set_control_pidRequest_(const ContainerAllocator& _alloc)
    : controlp(0.0)
    , controli(0.0)
    , controld(0.0)  {
    }

   typedef double  _controlp_type;
  _controlp_type controlp;

   typedef double  _controli_type;
  _controli_type controli;

   typedef double  _controld_type;
  _controld_type controld;



  typedef boost::shared_ptr< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::motor_set_control_pidRequest_<std::allocator<void> > motor_set_control_pidRequest;

typedef boost::shared_ptr< ::webots_ros::motor_set_control_pidRequest > motor_set_control_pidRequestPtr;
typedef boost::shared_ptr< ::webots_ros::motor_set_control_pidRequest const> motor_set_control_pidRequestConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace webots_ros

namespace ros
{
namespace message_traits
{

// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/groovy/share/std_msgs/msg'], 'webots_ros

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "712b4e401e3c9cbb098cd0435a9a13d3";
  }

  static const char* value(const ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9df5232b65af94fULL;
  static const uint64_t static_value2 = 0x73f79fe6d84301bbULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/motor_set_control_pidRequest";
  }

  static const char* value(const ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 controlp\n\
float64 controli\n\
float64 controld\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.controlp);
      stream.next(m.controli);
      stream.next(m.controld);

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
struct Printer< ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::motor_set_control_pidRequest_<ContainerAllocator>& v)
  {
        s << indent << "controlp: ";
    Printer<double>::stream(s, indent + "  ", v.controlp);
    s << indent << "controli: ";
    Printer<double>::stream(s, indent + "  ", v.controli);
    s << indent << "controld: ";
    Printer<double>::stream(s, indent + "  ", v.controld);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_MOTOR_SET_CONTROL_PIDREQUEST_H
