cmake_minimum_required(VERSION 3.16)
project(inv)
SET (G2O_LIBS g2o_csparse_extension g2o_cli g2o_core g2o_interface g2o_parser g2o_solver_cholmod
g2o_solver_csparse g2o_solver_dense g2o_solver_pcg g2o_stuff g2o_types_icp
g2o_types_sba g2o_types_sim3 g2o_types_slam2d g2o_types_slam3d glog)
add_executable(parsemap src/parsemap.cpp)
target_link_libraries(parsemap ${G2O_LIBS})
add_executable(rgb_slam src/rgb_slam.cpp)
target_link_libraries(rgb_slam ${G2O_LIBS})
add_executable(load_g2o src/load_g2o.cpp)
target_link_libraries(load_g2o ${G2O_LIBS})

# set(G2O_DIR "/home/ksuresh/g2o/cmake_modules")
# list(APPEND CMAKE_MODULE_PATH "/home/ksuresh/g2o/cmake_modules")
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(G2O g2o)
# link_libraries(${G2O_LIBRARIES})
# add_executable(parsemap parsemap.cpp)
# target_link_libraries(parsemap ${G2O_LIBRARIES})