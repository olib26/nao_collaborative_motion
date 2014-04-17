FILE(REMOVE_RECURSE
  "msg_gen"
  "src/first_approaches/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/first_approaches/PosImage.h"
  "msg_gen/cpp/include/first_approaches/Sonar.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
