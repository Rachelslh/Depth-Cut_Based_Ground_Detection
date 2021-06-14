cmake_minimum_required(VERSION 3.10.0)
project(Ground_Detection VERSION 0.1.0)

set(include include/ground_search.cpp)
add_executable(Main main.cpp ${include})

find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)

find_package(xtensor REQUIRED)
find_package(xtensor-blas REQUIRED)

find_package(OpenCV REQUIRED)

if (BLAS_FOUND AND LAPACK_FOUND AND OpenCV_FOUND)

  message(STATUS "BLAS VENDOR:    " ${BLA_VENDOR})
  message(STATUS "BLAS LIBRARIES: " ${BLAS_LIBRARIES})

  target_link_libraries(Main PUBLIC ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})

  target_include_directories(Main PUBLIC ${xtensor_INCLUDE_DIRS})
  target_link_libraries(Main PUBLIC xtensor)

  include_directories( ${OpenCV_INCLUDE_DIRS} )

  target_link_libraries( Main PUBLIC ${OpenCV_LIBS} )

endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
