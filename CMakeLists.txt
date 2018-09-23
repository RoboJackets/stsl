cmake_minimum_required(VERSION 3.10)
project(SoftwareTrainingSupportLibrary)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

set(LIB_SOURCE_FILES src/RJRobot.cpp)

add_library(STSL ${LIB_SOURCE_FILES})

if(WIN32)
    include(cmake/macros/get_WIN32_WINNT.cmake)
    get_WIN32_WINNT(ver)
    add_definitions(-D_WIN32_WINNT=${ver})
    target_link_libraries(STSL ws2_32 )
endif()

add_executable(system_test example/system_test.cpp)
target_link_libraries(system_test STSL)

install(TARGETS STSL
        ARCHIVE DESTINATION lib)

file(GLOB_RECURSE HDRS include/*.h)

install(FILES ${HDRS} DESTINATION include/STSL)
