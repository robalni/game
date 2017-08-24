if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (release or debug)" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_C_FLAGS "-std=c11 -Wall -Wextra ${CMAKE_C_FLAGS}")

set(sources main.c glad/src/glad.c)

include_directories(glad/include)

add_executable(a ${sources})

set(deps SDL2)

foreach(dep IN LISTS deps)
    find_package(${dep} REQUIRED)
    include_directories(${${dep}_INCLUDE_DIRS})
    target_link_libraries(a ${${dep}_LIBRARIES})
endforeach()
target_link_libraries(a dl m)
