cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  darknet_ros_msgs
  std_msgs
  image_transport
  cv_bridge
  message_generation
  geometry_msgs
  vision_msgs
  sensor_msgs
)
find_package(OpenCV REQUIRED)

add_message_files(FILES stopmsg.msg)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/home/tae/catkin_ws/src/VISION/src/parking_header")
include_directories("/home/tae/catkin_ws/src/VISION/src/vison_header")

add_message_files(
  FILES
    BoundingBox.msg
    BoundingBoxes.msg
    ObjectCount.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
  vision_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    message_runtime
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS
  darknet_ros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(vision_parking src/parking.cpp)
target_link_libraries(vision_parking ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(vision_publish src/vision_example_publish.cpp)
target_link_libraries(vision_publish ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(vision_stopline src/stopline.cpp)
target_link_libraries(vision_stopline ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(vision_detecting src/obj_detecting.cpp)
add_dependencies(vision_detecting ${PROJECT_NAME}_EXPORTED_TARGETS ${catkin_EXPORTED_TARGETS})
target_link_libraries(vision_detecting ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(find_cam_num src/find_cam_num.cpp)
target_link_libraries(find_cam_num ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(vision_subscribe src/vision_example_subscibe.cpp)
target_link_libraries(vision_subscribe ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(vision_example_yolov7_ros src/vision_example_yolov7_ros.cpp)
target_link_libraries(vision_example_yolov7_ros ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(back_parking src/back_parking.cpp)
target_link_libraries(back_parking ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(vision_parking2 src/parking2.cpp)
target_link_libraries(vision_parking2 ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(vision_detecting_yolov7 src/vision_detecting_yolov7.cpp)
target_link_libraries(vision_detecting_yolov7 ${catkin_LIBRARIES} ${OpenCV_LIBS})
