add_executable(aim main.cpp image_converter.cpp)
target_link_libraries(aim ${catkin_LIBRARIES} )
add_dependencies(aim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(aim ${OpenCV_LIBRARIES})
