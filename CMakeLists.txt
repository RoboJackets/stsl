cmake_minimum_required(VERSION 3.10)
project(SoftwareTrainingSupportLibrary)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "7")
        message(FATAL_ERROR "STSL requires version 7 or newer of the GNU compiler (GCC). You currently have version ${CMAKE_CXX_COMPILER_VERSION}")
    endif()
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    # TODO determine minimum version for C++17 compatibility
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS "3.9")
        message(FATAL_ERROR "STSL requires version 3.9 or newer of the Clang compiler. You currently have version ${CMAKE_CXX_COMPILER_VERSION}")
    endif()
else()
    message(WARNING "STSL does not currently support your selected compiler: ${CMAKE_CXX_COMPILER_ID}")
endif()

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

set(LIB_SOURCE_FILES src/RJRobot.cpp)

add_library(STSL ${LIB_SOURCE_FILES})

if(WIN32)
    include(cmake/macros/get_WIN32_WINNT.cmake)
    get_WIN32_WINNT(ver)
    add_definitions(-D_WIN32_WINNT=${ver})
    target_link_libraries(STSL ws2_32)
endif()

add_executable(Example example/main.cpp)
target_link_libraries(Example STSL)

install(TARGETS STSL
        ARCHIVE DESTINATION lib)

file(GLOB_RECURSE HDRS include/*.h)

install(FILES ${HDRS} DESTINATION include/STSL)
