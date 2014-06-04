FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/nao_behavior_tree/msg"
  "src/nao_behavior_tree/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/nao_behavior_tree/BallPosForHead.h"
  "srv_gen/cpp/include/nao_behavior_tree/GetArmReadyBT.h"
  "srv_gen/cpp/include/nao_behavior_tree/ArmReadyBT.h"
  "srv_gen/cpp/include/nao_behavior_tree/BallPosForHand.h"
  "srv_gen/cpp/include/nao_behavior_tree/BallPosGetBT.h"
  "srv_gen/cpp/include/nao_behavior_tree/BallPosChangeBT.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
