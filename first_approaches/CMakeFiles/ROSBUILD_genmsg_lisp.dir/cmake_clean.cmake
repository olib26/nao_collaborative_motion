FILE(REMOVE_RECURSE
  "msg_gen"
  "src/first_approaches/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/PosImage.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_PosImage.lisp"
  "msg_gen/lisp/Sonar.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_Sonar.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
