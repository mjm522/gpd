# Eigen library
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

#pcl
find_package(PCL 1.9 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#gpd
set(GPD_LIB "/usr/local/lib/libgpd.so")
set(GPD_INCLUDE "/usr/local/include/gpd")
include_directories(${GPD_INCLUDE})

#pybind
add_subdirectory(pybind11)

pybind11_add_module(gpd_py SHARED gpd_py_wrapper.cpp)
target_link_libraries(gpd_py PRIVATE ${GPD_LIB})

set_target_properties(gpd_py
  PROPERTIES
  SUFFIX ""
  VERSION "so"
  SOVERSION "so"
  DEFINE_SYMBOL _CINDEX_LIB_)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/gpd_py.so DESTINATION ${Python3_SITELIB})

FILE(COPY test.py DESTINATION .)
ADD_TEST(NAME GPDBoost COMMAND ${PYTHON_EXECUTABLE} test.py)