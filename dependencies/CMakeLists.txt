project(dependencies)

set(VCPKG_LIBRARIES_INCLUDE "D:/DevEnv/vcpkg/installed/x86-windows/include")
set(VCPKG_LIBRARIES_LIB "D:/DevEnv/vcpkg/installed/x86-windows/debug/lib;D:/DevEnv/vcpkg/installed/x86-windows/lib")

add_library(${PROJECT_NAME} INTERFACE)
add_library(module::dependencies ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  INTERFACE ${VCPKG_LIBRARIES_INCLUDE}
)
target_link_directories(${PROJECT_NAME}
  INTERFACE ${VCPKG_LIBRARIES_LIB}
)
