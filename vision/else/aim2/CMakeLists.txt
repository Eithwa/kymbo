add_executable(aim2 main2.cpp image_converter2.cpp)
target_link_libraries(aim2 ${catkin_LIBRARIES} )
add_dependencies(aim2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

find_package(OpenCV)
include_directories($ {} OpenCV_INCLUDE_DIRS)
target_link_libraries(aim2 ${OpenCV_LIBRARIES})
