cmake_minimum_required(VERSION 3.12)
cmake_policy(SET CMP0048 NEW)
project(xingtian_dynamics VERSION 1.0.0)

find_package(catkin REQUIRED)

# find_package(osqp REQUIRED)  
# include_directories()
# find_library(OSQP_LIBRARY osqp PATHS /usr/local/lib)
# find_path(OSQP_INCLUDE_DIR osqp.h PATHS /usr/local/include/osqp)

# find_package(osqp REQUIRED)
# message(STATUS "OSQP include directories: "${OSQP_INCLUDE_DIRS})
# message(STATUS "OSQP libraries: "${OSQP_LIBRARIES})
# 手动指定包含路径
include_directories(/usr/local/include)

# 手动指定库路径并链接 OSQP
# 增加src下的头文件
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/WBC)
# find_package(roslaunch)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslaunch
  controller_manager
  genmsg
  joint_state_controller
  robot_state_publisher
  gazebo_ros
  tf
  geometry_msgs
  xingtian_msgs
  genmsg
  # Eigen3

)
catkin_package(
  CATKIN_DEPENDS 
  # xingtian_msgs

)
set(CMAKE_BUILD_TYPE Debug)

set(CATKIN_MAKE ON)        # Use CATKIN_MAKE or not, ON or OFF
set(SIMULATION ON)         # Use Gazebo or not, ON or OFF
# add_library()
set(DEBUG ON)


# set(CMAKE_BUILD_TYPE Debug)
# 在 CMakeLists.txt 中添加
# add_executable(my_executable src/my_executable.cpp)
# target_include_directories(my_executable PRIVATE ${CMAKE_SOURCE_DIR}/../devel/include)
#添加C++文件
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    # ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}
    ${EIGEN_INCLUDE_DIRS}
    ${OSQP_INCLUDE_DIRS}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")
link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${OSQP_LIBRARY}
)
# add_executable(my_cpp_file src/main.cpp src/interface/sim_gazebo_ros.cpp)
file(GLOB_RECURSE SRC_LIST
   "src/*/*.cpp"
    "src/*/*.cc"
    "src/*/*/*.cpp"

)
add_executable(xingtian_dynamics
	src/main.cpp 
	${SRC_LIST}
	)

target_include_directories(xingtian_dynamics PRIVATE ${catkin_INCLUDE_DIRS})
target_include_directories(xingtian_dynamics PRIVATE ${OSQP_INCLUDE_DIR})
# target_link_libraries(xingtian_dynamics Eigen3::Eigen)

add_dependencies(xingtian_dynamics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xingtian_dynamics ${catkin_LIBRARIES} ${EIGEN_LIBRARIES})
  
# set_target_properties(xingtian_dynamics PROPERTIES COMPILE_FLAGS "-g")
target_link_libraries(xingtian_dynamics /usr/local/lib/libosqp.so)

target_link_libraries(xingtian_dynamics ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
# add_subdirectory(xingtian_msgs)

# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# target_link_libraries(xingtian_dynamics -pthread lcm)
foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
