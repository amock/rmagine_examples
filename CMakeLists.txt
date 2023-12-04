cmake_minimum_required(VERSION 3.4)
project(rmagine_examples)

include(GNUInstallDirs)
set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON)
set_property(GLOBAL PROPERTY USE_FOLDERS On)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(assimp REQUIRED)

find_package(rmagine 
COMPONENTS # required components 
    core
OPTIONAL_COMPONENTS # optional components. existence can be checked later
    embree
    cuda
    optix
)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

message(STATUS "Rmagine Found:")
message(STATUS "- version: ${rmagine_VERSION}")


# message(STATUS "- components: ${}")

# if(TARGET )


get_target_property(RMAGINE_CORE_INCLUDES rmagine::core INTERFACE_INCLUDE_DIRECTORIES)

# TODO: prepare example for component iteration
# foreach(component ${rmagine_FIND_COMPONENTS})

# endforeach()

message(STATUS "- includes: ${RMAGINE_CORE_INCLUDES}")

get_target_property(RMAGINE_CORE_INCLUDES rmagine::core INTERFACE_INCLUDE_DIRECTORIES)


message(STATUS "EIGEN_INCLUDES: ${EIGEN_INCLUDES}")
message(STATUS "RMAGINE_CORE_INCLUDES: ${RMAGINE_CORE_INCLUDES}")



message(STATUS "OLD STYLE CMAKE VARS:")
message(STATUS "- rmagine_INCLUDE_DIR: ${rmagine_INCLUDE_DIR}")
message(STATUS "- rmagine_LIBRARIES: ${rmagine_LIBRARIES}")

include_directories(include)

if(TARGET rmagine::core)

add_executable(example_math src/example_math.cpp)
target_link_libraries(example_math
    rmagine::core
)

endif(TARGET rmagine::core)

# Check if rmagine was compiled with embree support
if(TARGET rmagine::embree)

add_executable(example_cpu_sphere src/example_cpu_sphere.cpp)
target_link_libraries(example_cpu_sphere
    rmagine::embree
)

add_executable(example_cpu_pinhole src/example_cpu_pinhole.cpp)
target_link_libraries(example_cpu_pinhole
    rmagine::embree
)

add_executable(example_cpu_o1dn src/example_cpu_o1dn.cpp)
target_link_libraries(example_cpu_o1dn
    rmagine::embree
)

add_executable(example_cpu_ondn src/example_cpu_ondn.cpp)
target_link_libraries(example_cpu_ondn
    rmagine::embree
)

add_executable(example_cpu_change_map src/example_cpu_change_map.cpp)
target_link_libraries(example_cpu_change_map
    rmagine::embree
)

add_executable(example_cpu_change_sim src/example_cpu_change_sim.cpp)
target_link_libraries(example_cpu_change_sim
    rmagine::embree
)

endif(TARGET rmagine::embree)

# check if rmagine was compiled with optix support
if(TARGET rmagine::optix)

add_executable(example_gpu_sphere src/example_gpu_sphere.cpp)
target_link_libraries(example_gpu_sphere
    rmagine::optix
)

add_executable(example_gpu_pinhole src/example_gpu_pinhole.cpp)
target_link_libraries(example_gpu_pinhole
    rmagine::optix
)

add_executable(example_gpu_o1dn src/example_gpu_o1dn.cpp)
target_link_libraries(example_gpu_o1dn
    rmagine::optix
)

add_executable(example_gpu_ondn src/example_gpu_ondn.cpp)
target_link_libraries(example_gpu_ondn
    rmagine::optix
)

add_executable(example_gpu_change_map src/example_gpu_change_map.cpp)
target_link_libraries(example_gpu_change_map
    rmagine::optix
)

endif(TARGET rmagine::optix)
