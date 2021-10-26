cmake_minimum_required(VERSION 3.0.2)
project(mpc_controller)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

find_package(catkin_simple REQUIRED)
catkin_simple()

# activate c++ 11
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()


# flags for speed (should already be enabled by default)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fopenmp -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(ACADO REQUIRED)

cs_add_library(mpc_solver
  qpoases/SRC/Bounds.cpp
  qpoases/SRC/Constraints.cpp
  qpoases/SRC/CyclingManager.cpp
  qpoases/SRC/Indexlist.cpp
  qpoases/SRC/MessageHandling.cpp
  qpoases/SRC/QProblem.cpp
  qpoases/SRC/QProblemB.cpp
  qpoases/SRC/SubjectTo.cpp
  qpoases/SRC/Utils.cpp
  qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  lib/acado_qpoases_interface.cpp
  lib/acado_integrator.c
  lib/acado_solver.c
  lib/acado_auxiliary_functions.c
  # lib/acado_solver_mex.c
  # lib/acado_solver_sfunction.c
)

target_include_directories(mpc_solver PUBLIC
  # model/quadrotor_mpc_codegen/
  include/mpc_controller
  qpoases
  qpoases/INCLUDE
  qpoases/SRC)

# make an executable
cs_install()
cs_export()

include_directories(${EIGEN3_INCLUDE_DIRS})

# cs_add_executable(${PROJECT_NAME}_node src/mpc.cpp)

# target_link_libraries(${PROJECT_NAME}_node
#   # mpc_controller
#   # mpc_wrapper
#   mpc_solver)

# cs_install()
# cs_export()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpc_controller
  CATKIN_DEPENDS geometry_msgs mavros roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  . ${ACADO_INCLUDE_DIRS}
  # ~/catkin_ws/acado_mpc_export/
)

# vary time reference node
add_executable(${PROJECT_NAME}_node src/mpc.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${ACADO_SHARED_LIBRARIES}
  # libacado_exported_rti.a
  mpc_solver
)

# hover mpc node
# add_executable(hover_mpc_node src/mpc_hover.cpp)
# target_link_libraries(hover_mpc_node
#   ${catkin_LIBRARIES}
#   ${ACADO_SHARED_LIBRARIES}
#   # libacado_exported_rti.a
#   mpc_solver
# )