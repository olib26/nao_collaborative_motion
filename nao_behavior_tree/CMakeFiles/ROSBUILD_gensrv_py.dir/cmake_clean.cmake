FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/nao_behavior_tree/msg"
  "src/nao_behavior_tree/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/nao_behavior_tree/srv/__init__.py"
  "src/nao_behavior_tree/srv/_BallPosForHead.py"
  "src/nao_behavior_tree/srv/_GetArmReadyBT.py"
  "src/nao_behavior_tree/srv/_ArmReadyBT.py"
  "src/nao_behavior_tree/srv/_BallPosForHand.py"
  "src/nao_behavior_tree/srv/_BallPosGetBT.py"
  "src/nao_behavior_tree/srv/_BallPosChangeBT.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
