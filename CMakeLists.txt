cmake_minimum_required(VERSION 3.2)
project(rmagine_examples)


find_package(rmagine REQUIRED)
include_directories(${rmagine_INCLUDE_DIRS})

include_directories(include)

add_executable(example_cpu_sphere src/example_cpu_sphere.cpp)
target_link_libraries(example_cpu_sphere
    ${rmagine_LIBRARIES}
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

add_executable(example_gpu_sphere src/example_gpu_sphere.cpp)
target_link_libraries(example_gpu_sphere
    ${rmagine_LIBRARIES}
)

add_executable(example_gpu_pinhole src/example_gpu_pinhole.cpp)
target_link_libraries(example_gpu_pinhole
    ${rmagine_LIBRARIES}
)

add_executable(example_gpu_o1dn src/example_gpu_o1dn.cpp)
target_link_libraries(example_gpu_o1dn
    ${rmagine_LIBRARIES}
)

add_executable(example_gpu_ondn src/example_gpu_ondn.cpp)
target_link_libraries(example_gpu_ondn
    ${rmagine_LIBRARIES}
)

message(STATUS "${rmagine_LIBRARIES}")

add_executable(example_gpu_change_map src/example_gpu_change_map.cpp)
target_link_libraries(example_gpu_change_map
    ${rmagine_LIBRARIES}
)

target_compile_features(example_gpu_change_map
    PRIVATE cxx_std_17
)