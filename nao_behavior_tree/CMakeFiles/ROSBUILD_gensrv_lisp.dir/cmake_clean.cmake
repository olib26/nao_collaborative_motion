FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/nao_behavior_tree/msg"
  "src/nao_behavior_tree/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "srv_gen/lisp/BallPosForHead.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_BallPosForHead.lisp"
  "srv_gen/lisp/GetArmReadyBT.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_GetArmReadyBT.lisp"
  "srv_gen/lisp/ArmReadyBT.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_ArmReadyBT.lisp"
  "srv_gen/lisp/BallPosForHand.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_BallPosForHand.lisp"
  "srv_gen/lisp/BallPosGetBT.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_BallPosGetBT.lisp"
  "srv_gen/lisp/BallPosChangeBT.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_BallPosChangeBT.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
