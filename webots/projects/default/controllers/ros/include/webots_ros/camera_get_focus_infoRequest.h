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

#ifndef WEBOTS_ROS_MESSAGE_CAMERA_GET_FOCUS_INFOREQUEST_H
#define WEBOTS_ROS_MESSAGE_CAMERA_GET_FOCUS_INFOREQUEST_H

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
struct camera_get_focus_infoRequest_
{
  typedef camera_get_focus_infoRequest_<ContainerAllocator> Type;

  camera_get_focus_infoRequest_()
    : ask(0)  {
    }
  camera_get_focus_infoRequest_(const ContainerAllocator& _alloc)
    : ask(0)  {
    }

   typedef uint8_t  _ask_type;
  _ask_type ask;



  typedef boost::shared_ptr< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::camera_get_focus_infoRequest_<std::allocator<void> > camera_get_focus_infoRequest;

typedef boost::shared_ptr< ::webots_ros::camera_get_focus_infoRequest > camera_get_focus_infoRequestPtr;
typedef boost::shared_ptr< ::webots_ros::camera_get_focus_infoRequest const> camera_get_focus_infoRequestConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b515f92c1f91e19619caa437c4da4d85";
  }

  static const char* value(const ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9df5232b65af94fULL;
  static const uint64_t static_value2 = 0x73f79fe6d84301bbULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/camera_get_focus_infoRequest";
  }

  static const char* value(const ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 ask\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.ask);

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
struct Printer< ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::camera_get_focus_infoRequest_<ContainerAllocator>& v)
  {
        s << indent << "ask: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ask);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_CAMERA_GET_FOCUS_INFOREQUEST_H
