cmake_minimum_required(VERSION 3.7.2)
project(SoftwareTrainingSupportLibrary)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

set(LIB_SOURCE_FILES src/RJRobot.cpp)

add_library(STSL ${LIB_SOURCE_FILES})
target_link_libraries(STSL robotcontrol)

add_executable(Example example/main.cpp)
target_link_libraries(Example STSL)

install(TARGETS STSL
        ARCHIVE DESTINATION lib)

file(GLOB_RECURSE HDRS include/*.h)

install(FILES ${HDRS} DESTINATION include/STSL)

install(FILES "C++ (RoboJackets).run" 
	DESTINATION /home/debian/c9sdk/.c9/runners
	PERMISSIONS WORLD_READ WORLD_WRITE)

install(FILES rjresetworkspace.sh
        DESTINATION /usr/bin
        RENAME rjresetworkspace
        PERMISSIONS WORLD_READ WORLD_WRITE WORLD_EXECUTE)
