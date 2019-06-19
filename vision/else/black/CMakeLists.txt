add_executable(black main.cpp)
target_link_libraries(black ${catkin_LIBRARIES} )
add_dependencies(black ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(black ${OpenCV_LIBRARIES})
