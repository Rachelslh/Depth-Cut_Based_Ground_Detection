cmake_minimum_required(VERSION 3.10.0)
project(Ground_Detection VERSION 0.1.0)

set(include include/ground_search.cpp)
add_executable(Main main.cpp ${include})

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(OpenCV REQUIRED)

if (Eigen3_FOUND AND OpenCV_FOUND)

  target_link_libraries (Main PUBLIC Eigen3::Eigen)

  include_directories( ${EIGEN3_INCLUDE_DIR})

  include_directories( ${OpenCV_INCLUDE_DIRS} )

  target_link_libraries( Main PUBLIC ${OpenCV_LIBS} )

endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
