FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/nao_behavior_tree/msg"
  "src/nao_behavior_tree/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/test"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/test.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
