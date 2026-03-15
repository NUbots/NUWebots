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

#ifndef WEBOTS_ROS_MESSAGE_NODE_GET_FIELD_BY_INDEXREQUEST_H
#define WEBOTS_ROS_MESSAGE_NODE_GET_FIELD_BY_INDEXREQUEST_H

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
struct node_get_field_by_indexRequest_
{
  typedef node_get_field_by_indexRequest_<ContainerAllocator> Type;

  node_get_field_by_indexRequest_()
    : node(0)
    , index(0)
    , proto(false)  {
    }
  node_get_field_by_indexRequest_(const ContainerAllocator& _alloc)
    : node(0)
    , index(0)
    , proto(false)  {
    }

   typedef uint64_t  _node_type;
  _node_type node;

   typedef uint32_t  _index_type;
  _index_type index;

   typedef uint8_t  _proto_type;
  _proto_type proto;



  typedef boost::shared_ptr< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::node_get_field_by_indexRequest_<std::allocator<void> > node_get_field_by_indexRequest;

typedef boost::shared_ptr< ::webots_ros::node_get_field_by_indexRequest > node_get_field_by_indexRequestPtr;
typedef boost::shared_ptr< ::webots_ros::node_get_field_by_indexRequest const> node_get_field_by_indexRequestConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "24bbfc7fa321c886f58114d630e6f1fb";
  }

  static const char* value(const ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9df5232b65af94fULL;
  static const uint64_t static_value2 = 0x73f79fe6d84301bbULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/node_get_field_by_indexRequest";
  }

  static const char* value(const ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 node\n\
uint32 index\n\
bool proto\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.node);
      stream.next(m.index);
      stream.next(m.proto);

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
struct Printer< ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::node_get_field_by_indexRequest_<ContainerAllocator>& v)
  {
        s << indent << "node: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.node);
    s << indent << "index: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.index);
    s << indent << "proto: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.proto);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_NODE_GET_FIELD_BY_INDEXREQUEST_H
