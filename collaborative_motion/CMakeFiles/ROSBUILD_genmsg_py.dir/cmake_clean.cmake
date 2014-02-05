FILE(REMOVE_RECURSE
  "msg_gen"
  "src/collaborative_motion/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/collaborative_motion/msg/__init__.py"
  "src/collaborative_motion/msg/_PosImage.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
