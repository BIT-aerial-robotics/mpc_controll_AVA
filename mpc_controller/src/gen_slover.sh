#! /bin/bash
echo "first of all, make static library"
cd ~/catkin_ws/acado_mpc_export
make
echo "finish make file"
cp -f ~/catkin_ws/acado_mpc_export/acado_solver_sfunction.c ~/catkin_ws/src/mpc_controller/lib
cp -f libacado_exported_rti.a acado_auxiliary_functions.c acado_integrator.c acado_qpoases_interface.cpp acado_qpoases_interface.hpp acado_solver.c acado_solver_mex.c ~/catkin_ws/src/mpc_controller/lib
echo "finish copy source file to destination path"
cp -f acado_solver_sfunction.h acado_auxiliary_functions.h acado_common.h acado_qpoases_interface.hpp ~/catkin_ws/src/mpc_controller/include/mpc_controller
echo "finish copy include file to destination"
echo "finish all..., in order to use it, you need to rebuild your ros package"
cd ~/catkin_ws/
catkin build mpc_controller
