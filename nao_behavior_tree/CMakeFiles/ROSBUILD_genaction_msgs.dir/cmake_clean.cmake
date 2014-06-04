FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/nao_behavior_tree/msg"
  "src/nao_behavior_tree/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
