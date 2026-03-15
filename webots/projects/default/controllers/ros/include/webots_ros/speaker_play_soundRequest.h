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

#ifndef WEBOTS_ROS_MESSAGE_SPEAKER_PLAY_SOUNDREQUEST_H
#define WEBOTS_ROS_MESSAGE_SPEAKER_PLAY_SOUNDREQUEST_H

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
struct speaker_play_soundRequest_
{
  typedef speaker_play_soundRequest_<ContainerAllocator> Type;

  speaker_play_soundRequest_()
    : sound()
    , volume(0.0)
    , pitch(0.0)
    , balance(0.0)
    , loop(0)  {
    }
  speaker_play_soundRequest_(const ContainerAllocator& _alloc)
    : sound(_alloc)
    , volume(0.0)
    , pitch(0.0)
    , balance(0.0)
    , loop(0)  {
    }

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >   _sound_type;
  _sound_type sound;

   typedef double  _volume_type;
  _volume_type volume;

   typedef double  _pitch_type;
  _pitch_type pitch;

   typedef double  _balance_type;
  _balance_type balance;

   typedef int8_t  _loop_type;
  _loop_type loop;



  typedef boost::shared_ptr< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

};

typedef ::webots_ros::speaker_play_soundRequest_<std::allocator<void> > speaker_play_soundRequest;

typedef boost::shared_ptr< ::webots_ros::speaker_play_soundRequest > speaker_play_soundRequestPtr;
typedef boost::shared_ptr< ::webots_ros::speaker_play_soundRequest const> speaker_play_soundRequestConstPtr;

// constants requiring out of line definition

template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> const>
  : FalseType
  { };

template<class ContainerAllocator>
struct MD5Sum< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9c17e6742fccca17f3542e68a9800dd3";
  }

  static const char* value(const ::webots_ros::speaker_play_soundRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf9df5232b65af94fULL;
  static const uint64_t static_value2 = 0x73f79fe6d84301bbULL;
};

template<class ContainerAllocator>
struct DataType< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "webots_ros/speaker_play_soundRequest";
  }

  static const char* value(const ::webots_ros::speaker_play_soundRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string sound\n\
float64 volume\n\
float64 pitch\n\
float64 balance\n\
int8 loop\n\\n\
\n\
";
  }

  static const char* value(const ::webots_ros::speaker_play_soundRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
            stream.next(m.sound);
      stream.next(m.volume);
      stream.next(m.pitch);
      stream.next(m.balance);
      stream.next(m.loop);

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
struct Printer< ::webots_ros::speaker_play_soundRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::webots_ros::speaker_play_soundRequest_<ContainerAllocator>& v)
  {
        s << indent << "sound: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.sound);
    s << indent << "volume: ";
    Printer<double>::stream(s, indent + "  ", v.volume);
    s << indent << "pitch: ";
    Printer<double>::stream(s, indent + "  ", v.pitch);
    s << indent << "balance: ";
    Printer<double>::stream(s, indent + "  ", v.balance);
    s << indent << "loop: ";
    Printer<int8_t>::stream(s, indent + "  ", v.loop);

  }
};

} // namespace message_operations
} // namespace ros

#endif // WEBOTS_ROS_MESSAGE_SPEAKER_PLAY_SOUNDREQUEST_H
