project(encode_video)

add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME}
  PRIVATE main.cc
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE module::dependencies
  PRIVATE avcodec avutil
)