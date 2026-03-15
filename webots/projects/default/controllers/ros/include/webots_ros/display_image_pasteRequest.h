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

#ifndef WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_PASTEREQUEST_H
#define WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_PASTEREQUEST_H

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
struct display_image_pasteRequest_
{
  typedef display_image_pasteRequest_<ContainerAllocator> Type;

  display_image_pasteRequest_()
    : ir(0)
    , x(0)
    , y(0)
    , blend(0)  {
    }
  display_image_pasteRequest_(const ContainerAllocator& _alloc)
    : ir(0)
    , x(0)
    , y(0)
    , blend(0)  {
    }

   typedef uint64_t  _ir_type;
  _ir_type ir;

   typedef int32_t  _x_type;
  _x_type x;

   typedef int32_t  _y_type;
  _y_type y;

   typedef uint8_t  _blend_type;
  _blend_type blend;



  typedef boost::shared_ptr< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::display_image_pasteRequest_<std::allocator<void> > display_image_pasteRequest;

typedef boost::shared_ptr< ::webots_ros::display_image_pasteRequest > display_image_pasteRequestPtr;
typedef boost::shared_ptr< ::webots_ros::display_image_pasteRequest const> display_image_pasteRequestConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::display_image_pasteRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "58100642be61cff1b755b0787ba1decc";
  }

  static const char* value(const ::webots_ros::display_image_pasteRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9df5232b65af94fULL;
  static const uint64_t static_value2 = 0x73f79fe6d84301bbULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/display_image_pasteRequest";
  }

  static const char* value(const ::webots_ros::display_image_pasteRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint64 ir\n\
int32 x\n\
int32 y\n\
uint8 blend\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::display_image_pasteRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.ir);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.blend);

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
struct Printer< ::webots_ros::display_image_pasteRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::display_image_pasteRequest_<ContainerAllocator>& v)
  {
        s << indent << "ir: ";
    Printer<uint64_t>::stream(s, indent + "  ", v.ir);
    s << indent << "x: ";
    Printer<int32_t>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<int32_t>::stream(s, indent + "  ", v.y);
    s << indent << "blend: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.blend);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_DISPLAY_IMAGE_PASTEREQUEST_H
