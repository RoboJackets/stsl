cmake_minimum_required(VERSION 3.5)
project(gpiod)

include(ExternalProject)

find_package(ament_cmake REQUIRED)

set(libgpiod_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libgpiod_external)

set(libgpiod_LIBRARIES ${CMAKE_INSTALL_PREFIX}/lib/libgpiodcxx.so)

ExternalProject_Add(libgpiod
  SOURCE_DIR ${libgpiod_SOURCE_DIR}
  GIT_REPOSITORY    git://git.kernel.org/pub/scm/libs/libgpiod/libgpiod.git
  GIT_TAG           v1.4.1
  CONFIGURE_COMMAND ${libgpiod_SOURCE_DIR}/autogen.sh --enable-bindings-cxx --prefix=${CMAKE_INSTALL_PREFIX} && make clean
  BUILD_COMMAND make
  INSTALL_COMMAND make install
)

add_library(gpiodcxx INTERFACE)
add_dependencies(gpiodcxx libgpiod)
target_link_libraries(gpiodcxx INTERFACE ${libgpiod_LIBRARIES})
target_include_directories(gpiodcxx INTERFACE ${CMAKE_INSTALL_PREFIX}/include)

ament_export_interfaces(export_${PROJECT_NAME})

ament_package()

install(
  TARGETS gpiodcxx
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
