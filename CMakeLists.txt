cmake_minimum_required(VERSION 2.8.3)
project(speed_planner)

add_compile_options(-std=c++11)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -std=c++11 -g -D_GLIBCXX_USE_CXX11_ABI=0")
#set(GUROBI_INCLUDE_DIRECTORY "/opt/gurobi811/linux64/include")
#set(GUROBI_LIB_DIRECTORY "/opt/gurobi811/linux64/lib")


find_package(Eigen3 REQUIRED)



include_directories(
  include
  #${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  #${GUROBI_INCLUDE_DIRECTORY}
)

#link_directories(${GUROBI_LIB_DIRECTORY})

add_executable(speed_planner src/speed_planner_node.cpp src/convex_speed_optimizer.cpp src/collision_checker.cpp src/utils.cpp src/obstacle.cpp)
#target_link_libraries (speed_planner gurobi_c++ gurobi81 ${catkin_LIBRARIES} /opt/gurobi811/linux64/lib/libgurobi_g++5.2.a)
#target_link_libraries (speed_planner ${catkin_LIBRARIES} /opt/gurobi811/linux64/lib/libgurobi_g++5.2.a /opt/gurobi811/linux64/lib/libgurobi81.so)
#add_dependencies(speed_planner ${catkin_EXPORTED_TARGETS})



