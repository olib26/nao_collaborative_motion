/* Auto-generated by genmsg_cpp for file /home/olivier/ros_workspace/nao_behavior_tree/msg/ROSActionGoal.msg */
#ifndef NAO_BEHAVIOR_TREE_MESSAGE_ROSACTIONGOAL_H
#define NAO_BEHAVIOR_TREE_MESSAGE_ROSACTIONGOAL_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "nao_behavior_tree/ROSGoal.h"

namespace nao_behavior_tree
{
template <class ContainerAllocator>
struct ROSActionGoal_ {
  typedef ROSActionGoal_<ContainerAllocator> Type;

  ROSActionGoal_()
  : header()
  , goal_id()
  , goal()
  {
  }

  ROSActionGoal_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , goal_id(_alloc)
  , goal(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
   ::actionlib_msgs::GoalID_<ContainerAllocator>  goal_id;

  typedef  ::nao_behavior_tree::ROSGoal_<ContainerAllocator>  _goal_type;
   ::nao_behavior_tree::ROSGoal_<ContainerAllocator>  goal;


  typedef boost::shared_ptr< ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ROSActionGoal
typedef  ::nao_behavior_tree::ROSActionGoal_<std::allocator<void> > ROSActionGoal;

typedef boost::shared_ptr< ::nao_behavior_tree::ROSActionGoal> ROSActionGoalPtr;
typedef boost::shared_ptr< ::nao_behavior_tree::ROSActionGoal const> ROSActionGoalConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace nao_behavior_tree

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e5cdf3142d0d3a2f937b2ba98f357651";
  }

  static const char* value(const  ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe5cdf3142d0d3a2fULL;
  static const uint64_t static_value2 = 0x937b2ba98f357651ULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/ROSActionGoal";
  }

  static const char* value(const  ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
ROSGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: nao_behavior_tree/ROSGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#goal definition\n\
int32 GOAL_\n\
\n\
";
  }

  static const char* value(const  ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.goal_id);
    stream.next(m.goal);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ROSActionGoal_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::nao_behavior_tree::ROSActionGoal_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
s << std::endl;
    Printer< ::nao_behavior_tree::ROSGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};


} // namespace message_operations
} // namespace ros

#endif // NAO_BEHAVIOR_TREE_MESSAGE_ROSACTIONGOAL_H

