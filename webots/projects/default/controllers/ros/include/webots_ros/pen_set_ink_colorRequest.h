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

#ifndef WEBOTS_ROS_MESSAGE_PEN_SET_INK_COLORREQUEST_H
#define WEBOTS_ROS_MESSAGE_PEN_SET_INK_COLORREQUEST_H

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
struct pen_set_ink_colorRequest_
{
  typedef pen_set_ink_colorRequest_<ContainerAllocator> Type;

  pen_set_ink_colorRequest_()
    : color(0)
    , density(0.0)  {
    }
  pen_set_ink_colorRequest_(const ContainerAllocator& _alloc)
    : color(0)
    , density(0.0)  {
    }

   typedef int32_t  _color_type;
  _color_type color;

   typedef double  _density_type;
  _density_type density;



  typedef boost::shared_ptr< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::pen_set_ink_colorRequest_<std::allocator<void> > pen_set_ink_colorRequest;

typedef boost::shared_ptr< ::webots_ros::pen_set_ink_colorRequest > pen_set_ink_colorRequestPtr;
typedef boost::shared_ptr< ::webots_ros::pen_set_ink_colorRequest const> pen_set_ink_colorRequestConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7fde2080d14960fbc22200121684eabf";
  }

  static const char* value(const ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9df5232b65af94fULL;
  static const uint64_t static_value2 = 0x73f79fe6d84301bbULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/pen_set_ink_colorRequest";
  }

  static const char* value(const ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 color\n\
float64 density\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.color);
      stream.next(m.density);

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
struct Printer< ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::pen_set_ink_colorRequest_<ContainerAllocator>& v)
  {
        s << indent << "color: ";
    Printer<int32_t>::stream(s, indent + "  ", v.color);
    s << indent << "density: ";
    Printer<double>::stream(s, indent + "  ", v.density);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_PEN_SET_INK_COLORREQUEST_H
