cmake_minimum_required(VERSION 3.2)
project(imagine_examples)


find_package(imagine REQUIRED)
include_directories(${imagine_INCLUDE_DIRS})

add_executable(example_cpu src/example_cpu.cpp)
target_link_libraries(example_cpu
    ${imagine_LIBRARIES}
)

add_executable(example_gpu src/example_gpu.cpp)
target_link_libraries(example_gpu
    ${imagine_LIBRARIES}
)

add_executable(example_cpu_custom_model src/example_cpu_custom_model.cpp)
target_link_libraries(example_cpu_custom_model
    ${imagine_LIBRARIES}
)

add_executable(example_gpu_custom_model src/example_gpu_custom_model.cpp)
target_link_libraries(example_gpu_custom_model
    ${imagine_LIBRARIES}
)