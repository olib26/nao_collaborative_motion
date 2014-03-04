FILE(REMOVE_RECURSE
  "msg_gen"
  "src/nao_behavior_tree/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/nao_behavior_tree/msg/__init__.py"
  "src/nao_behavior_tree/msg/_ROSAction.py"
  "src/nao_behavior_tree/msg/_ROSGoal.py"
  "src/nao_behavior_tree/msg/_ROSActionGoal.py"
  "src/nao_behavior_tree/msg/_ROSResult.py"
  "src/nao_behavior_tree/msg/_ROSActionResult.py"
  "src/nao_behavior_tree/msg/_ROSFeedback.py"
  "src/nao_behavior_tree/msg/_ROSActionFeedback.py"
  "src/nao_behavior_tree/msg/_Sonar.py"
  "src/nao_behavior_tree/msg/_Odometry.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
