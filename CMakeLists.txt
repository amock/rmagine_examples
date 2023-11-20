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

include_directories(include)

# Check if rmagine was compiled with embree support
if(TARGET rmagine::embree)

add_executable(example_cpu_sphere src/example_cpu_sphere.cpp)
target_link_libraries(example_cpu_sphere
    rmagine::embree
)

add_executable(example_cpu_pinhole src/example_cpu_pinhole.cpp)
target_link_libraries(example_cpu_pinhole
    ${rmagine_LIBRARIES}
)

add_executable(example_cpu_o1dn src/example_cpu_o1dn.cpp)
target_link_libraries(example_cpu_o1dn
    ${rmagine_LIBRARIES}
)

add_executable(example_cpu_ondn src/example_cpu_ondn.cpp)
target_link_libraries(example_cpu_ondn
    ${rmagine_LIBRARIES}
)

add_executable(example_cpu_change_map src/example_cpu_change_map.cpp)
target_link_libraries(example_cpu_change_map
    ${rmagine_LIBRARIES}
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
