/* Auto-generated by genmsg_cpp for file /home/olivier/ros_workspace/nao_behavior_tree/srv/BallPosForHead.srv */
#ifndef NAO_BEHAVIOR_TREE_SERVICE_BALLPOSFORHEAD_H
#define NAO_BEHAVIOR_TREE_SERVICE_BALLPOSFORHEAD_H
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

#include "ros/service_traits.h"




namespace nao_behavior_tree
{
template <class ContainerAllocator>
struct BallPosForHeadRequest_ {
  typedef BallPosForHeadRequest_<ContainerAllocator> Type;

  BallPosForHeadRequest_()
  : min_h(0)
  , max_h(0)
  {
  }

  BallPosForHeadRequest_(const ContainerAllocator& _alloc)
  : min_h(0)
  , max_h(0)
  {
  }

  typedef int32_t _min_h_type;
  int32_t min_h;

  typedef int32_t _max_h_type;
  int32_t max_h;


  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BallPosForHeadRequest
typedef  ::nao_behavior_tree::BallPosForHeadRequest_<std::allocator<void> > BallPosForHeadRequest;

typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHeadRequest> BallPosForHeadRequestPtr;
typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHeadRequest const> BallPosForHeadRequestConstPtr;


template <class ContainerAllocator>
struct BallPosForHeadResponse_ {
  typedef BallPosForHeadResponse_<ContainerAllocator> Type;

  BallPosForHeadResponse_()
  : pos(0)
  {
  }

  BallPosForHeadResponse_(const ContainerAllocator& _alloc)
  : pos(0)
  {
  }

  typedef int64_t _pos_type;
  int64_t pos;


  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BallPosForHeadResponse
typedef  ::nao_behavior_tree::BallPosForHeadResponse_<std::allocator<void> > BallPosForHeadResponse;

typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHeadResponse> BallPosForHeadResponsePtr;
typedef boost::shared_ptr< ::nao_behavior_tree::BallPosForHeadResponse const> BallPosForHeadResponseConstPtr;

struct BallPosForHead
{

typedef BallPosForHeadRequest Request;
typedef BallPosForHeadResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct BallPosForHead
} // namespace nao_behavior_tree

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "6fbdb3f4f625c750fdfa49da0b25855c";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x6fbdb3f4f625c750ULL;
  static const uint64_t static_value2 = 0xfdfa49da0b25855cULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosForHeadRequest";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 min_h\n\
int32 max_h\n\
\n\
";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "ebf28f8999fe57e1eb3bfb21c1acfef7";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xebf28f8999fe57e1ULL;
  static const uint64_t static_value2 = 0xeb3bfb21c1acfef7ULL;
};

template<class ContainerAllocator>
struct DataType< ::nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosForHeadResponse";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int64 pos\n\
\n\
\n\
";
  }

  static const char* value(const  ::nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.min_h);
    stream.next(m.max_h);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BallPosForHeadRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.pos);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BallPosForHeadResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<nao_behavior_tree::BallPosForHead> {
  static const char* value() 
  {
    return "dc8602d6c6bd69c5f7e48c7d10bea4cc";
  }

  static const char* value(const nao_behavior_tree::BallPosForHead&) { return value(); } 
};

template<>
struct DataType<nao_behavior_tree::BallPosForHead> {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosForHead";
  }

  static const char* value(const nao_behavior_tree::BallPosForHead&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dc8602d6c6bd69c5f7e48c7d10bea4cc";
  }

  static const char* value(const nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosForHead";
  }

  static const char* value(const nao_behavior_tree::BallPosForHeadRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "dc8602d6c6bd69c5f7e48c7d10bea4cc";
  }

  static const char* value(const nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "nao_behavior_tree/BallPosForHead";
  }

  static const char* value(const nao_behavior_tree::BallPosForHeadResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // NAO_BEHAVIOR_TREE_SERVICE_BALLPOSFORHEAD_H

