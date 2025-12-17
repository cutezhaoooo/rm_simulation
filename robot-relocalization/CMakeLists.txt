cmake_minimum_required(VERSION 3.5)
project(bbs_based_initializer)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
option(BUILD_CUDA "Build GPU version" ON)

set(CMAKE_CUDA_HOST_COMPILER /usr/bin/g++-10)


# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Auto-detect CUDA availability
if(BUILD_CUDA)
  # Check for CUDA compiler and runtime
  find_package(CUDAToolkit QUIET)
  if(CUDAToolkit_FOUND)
    # Check if we can enable CUDA language
    try_compile(CUDA_LANGUAGE_SUPPORTED
      ${CMAKE_BINARY_DIR}/cuda_language_test
      ${CMAKE_CURRENT_SOURCE_DIR}/cmake/test_cuda_runtime.cpp
      LINK_LIBRARIES CUDA::cudart
      OUTPUT_VARIABLE CUDA_COMPILE_OUTPUT
    )
    
    if(CUDA_LANGUAGE_SUPPORTED)
      enable_language(CUDA)
      set(USE_CUDA ON)
      add_definitions(-DUSE_CUDA)
      
      # Set CUDA architectures to avoid policy warning
      if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
        set(CMAKE_CUDA_ARCHITECTURES "75;80;86" CACHE STRING "CUDA architectures")
      endif()
      
      message(STATUS "CUDA toolkit detected, building GPU version")
      message(STATUS "CUDA version: ${CUDAToolkit_VERSION}")
    else()
      set(USE_CUDA OFF)
      message(STATUS "CUDA toolkit found but compilation test failed, building CPU version")
      message(STATUS "CUDA compile output: ${CUDA_COMPILE_OUTPUT}")
    endif()
  else()
    set(USE_CUDA OFF)
    message(STATUS "CUDA toolkit not found, building CPU version")
  endif()
else()
  set(USE_CUDA OFF)
  message(STATUS "CUDA disabled by BUILD_CUDA=OFF, building CPU version")
endif()

# Define source files
set(BBS_INITIALIZER_SOURCES
  src/bbs_based_initializer/bbs_based_initializer.cpp
  src/bbs_based_initializer/bbs_based_initializer_node.cpp
)

set(CPU_BBS3D_SOURCES
  src/cpu_bbs3d/bbs3d.cpp
  src/cpu_bbs3d/voxelmaps.cpp
  src/cpu_bbs3d/voxelmaps_io.cpp
)

set(GPU_BBS3D_SOURCES
  src/gpu_bbs3d/bbs3d.cu
  src/gpu_bbs3d/calc_score.cu
  src/gpu_bbs3d/voxelmaps.cu
  src/gpu_bbs3d/voxelmaps_io.cu
  src/gpu_bbs3d/stream_manager/check_error.cu
)

# Create main executable
if(USE_CUDA)
  # GPU version
  ament_auto_add_executable(bbs_based_initializer
    ${BBS_INITIALIZER_SOURCES}
    ${GPU_BBS3D_SOURCES}
  )
  
  set_property(TARGET bbs_based_initializer PROPERTY CUDA_STANDARD 17)
  set_property(TARGET bbs_based_initializer PROPERTY CUDA_STANDARD_REQUIRED ON)
  
  # Link CUDA libraries
  target_link_libraries(bbs_based_initializer
    ${PCL_LIBRARIES}
    yaml-cpp
    CUDA::cudart
  )
  
  # Include CUDA headers
  target_include_directories(bbs_based_initializer PRIVATE
    ${CUDAToolkit_INCLUDE_DIRS}
  )
else()
  # CPU version
  ament_auto_add_executable(bbs_based_initializer
    ${BBS_INITIALIZER_SOURCES}
    ${CPU_BBS3D_SOURCES}
  )
  
  target_link_libraries(bbs_based_initializer
    ${PCL_LIBRARIES}
    yaml-cpp
  )
endif()

# Include directories
target_include_directories(bbs_based_initializer
  PUBLIC 
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# OpenMP for CPU parallelization
find_package(OpenMP)
if(OpenMP_CXX_FOUND AND NOT USE_CUDA)
  target_link_libraries(bbs_based_initializer OpenMP::OpenMP_CXX)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
