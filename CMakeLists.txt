cmake_minimum_required(VERSION 2.8.3)
project(grasp_data_generator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
add_definitions(-w) # 忽略警告


find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


# PCL
find_package(PCL REQUIRED)
#find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
)

aux_source_directory(./src/grasp_detect/candidate candidate_src)
aux_source_directory(./src/grasp_detect/util util_src)


## generate dataset
add_executable(generate_predict_dataset src/app/generate_grasp_dataset.cpp
  src/grasp_detect/grasp_dataset_generate.cpp

  ${candidate_src}
  ${util_src}
) 
target_link_libraries(generate_predict_dataset
  

  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(generate_classify_dataset src/app/generate_classify_dataset.cpp
  src/grasp_detect/grasp_classify_dataset_generate.cpp

  ${candidate_src}
  ${util_src}
) 
target_link_libraries(generate_classify_dataset
  

  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(read_grasp_txt src/app/read_grasp_from_txt.cpp
  # src/grasp_detect/grasp_detector_pointnetgpd read_grasp_from_net

  ${candidate_src}
  ${util_src}
)
target_link_libraries(read_grasp_txt
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(read_grasp_net src/app/read_grasp_from_net.cpp
  # src/grasp_detect/grasp_detector_pointnetgpd read_grasp_from_net

  ${candidate_src}
  ${util_src}
)
target_link_libraries(read_grasp_net
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
