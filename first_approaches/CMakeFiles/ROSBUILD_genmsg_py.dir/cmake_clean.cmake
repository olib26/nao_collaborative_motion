FILE(REMOVE_RECURSE
  "msg_gen"
  "src/first_approaches/msg"
  "msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/first_approaches/msg/__init__.py"
  "src/first_approaches/msg/_PosImage.py"
  "src/first_approaches/msg/_Sonar.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
