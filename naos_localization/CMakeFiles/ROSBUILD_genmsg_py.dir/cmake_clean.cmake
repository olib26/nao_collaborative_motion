FILE(REMOVE_RECURSE
  "msg_gen"
  "src/naos_localization/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/naos_localization/msg/__init__.py"
  "src/naos_localization/msg/_Odometry.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
