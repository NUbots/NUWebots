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

#ifndef WEBOTS_ROS_MESSAGE_SKIN_GET_BONE_POSITIONREQUEST_H
#define WEBOTS_ROS_MESSAGE_SKIN_GET_BONE_POSITIONREQUEST_H

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
struct skin_get_bone_positionRequest_
{
  typedef skin_get_bone_positionRequest_<ContainerAllocator> Type;

  skin_get_bone_positionRequest_()
    : index(0)
    , absolute(false)  {
    }
  skin_get_bone_positionRequest_(const ContainerAllocator& _alloc)
    : index(0)
    , absolute(false)  {
    }

   typedef int32_t  _index_type;
  _index_type index;

   typedef uint8_t  _absolute_type;
  _absolute_type absolute;



  typedef boost::shared_ptr< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::skin_get_bone_positionRequest_<std::allocator<void> > skin_get_bone_positionRequest;

typedef boost::shared_ptr< ::webots_ros::skin_get_bone_positionRequest > skin_get_bone_positionRequestPtr;
typedef boost::shared_ptr< ::webots_ros::skin_get_bone_positionRequest const> skin_get_bone_positionRequestConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c9870aedfb26769e03d175a5a34b1ff2";
  }

  static const char* value(const ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9df5232b65af94fULL;
  static const uint64_t static_value2 = 0x73f79fe6d84301bbULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/skin_get_bone_positionRequest";
  }

  static const char* value(const ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 index\n\
bool absolute\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.index);
      stream.next(m.absolute);

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
struct Printer< ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::skin_get_bone_positionRequest_<ContainerAllocator>& v)
  {
        s << indent << "index: ";
    Printer<int32_t>::stream(s, indent + "  ", v.index);
    s << indent << "absolute: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.absolute);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SKIN_GET_BONE_POSITIONREQUEST_H
