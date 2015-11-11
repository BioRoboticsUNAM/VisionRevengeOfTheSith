cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  visualization_msgs
  message_generation
)

find_package(OpenCV REQUIRED)
#find_package(CUDA REQUIRED)

#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS})
#SET (CUDA_VERBOSE_BUILD ON CHACHE BOOL "nvcc verbose" FORCE)
#SET (LIB_TYPE STATIC)

add_service_files(
   FILES
   vsn_findonplanes.srv
)

generate_messages(
   DEPENDENCIES
   geometry_msgs   std_msgs   visualization_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#CUDA_ADD_LIBRARY(clusterizer_cu ${LIB_TYPE} src/clusterizer.cu)
#add_definitions(${PCL_DEFINITIONS})

add_executable(vision_node 
  src/vision_node.cpp
  src/Transform/RobotBase.cpp
  src/Transform/Head.cpp
  src/Transform/Transform.cpp
  src/Navigation/OccupGrid.cpp
  src/Utils/Plane3D.cpp
  src/Utils/PlanarHorizontalSegment.cpp
  src/Utils/DetectedObject.cpp
  src/Utils/ObjectsExtractor.cpp
  src/PlaneExtraction/PlaneExtractor.cpp
)

#ADD_DEPENDENCIES(visual_node clusterizer_cu)

target_link_libraries(vision_node
  ${OpenCV_LIBS}
   ${catkin_LIBRARIES}
)
