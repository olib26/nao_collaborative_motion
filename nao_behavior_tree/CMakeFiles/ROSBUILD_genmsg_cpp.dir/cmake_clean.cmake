FILE(REMOVE_RECURSE
  "msg_gen"
  "src/nao_behavior_tree/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/nao_behavior_tree/ROSAction.h"
  "msg_gen/cpp/include/nao_behavior_tree/ROSGoal.h"
  "msg_gen/cpp/include/nao_behavior_tree/ROSActionGoal.h"
  "msg_gen/cpp/include/nao_behavior_tree/ROSResult.h"
  "msg_gen/cpp/include/nao_behavior_tree/ROSActionResult.h"
  "msg_gen/cpp/include/nao_behavior_tree/ROSFeedback.h"
  "msg_gen/cpp/include/nao_behavior_tree/ROSActionFeedback.h"
  "msg_gen/cpp/include/nao_behavior_tree/Sonar.h"
  "msg/ROSAction.msg"
  "msg/ROSGoal.msg"
  "msg/ROSActionGoal.msg"
  "msg/ROSResult.msg"
  "msg/ROSActionResult.msg"
  "msg/ROSFeedback.msg"
  "msg/ROSActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
