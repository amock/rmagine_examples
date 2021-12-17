cmake_minimum_required(VERSION 3.2)
project(imagine_examples)


find_package(imagine REQUIRED)
include_directories(${imagine_INCLUDE_DIRS})

include_directories(include)

add_executable(example_cpu_sphere src/example_cpu_sphere.cpp)
target_link_libraries(example_cpu_sphere
    ${imagine_LIBRARIES}
)

add_executable(example_cpu_pinhole src/example_cpu_pinhole.cpp)
target_link_libraries(example_cpu_pinhole
    ${imagine_LIBRARIES}
)


add_executable(example_cpu_o1dn src/example_cpu_o1dn.cpp)
target_link_libraries(example_cpu_o1dn
    ${imagine_LIBRARIES}
)

add_executable(example_gpu_sphere src/example_gpu_sphere.cpp)
target_link_libraries(example_gpu_sphere
    ${imagine_LIBRARIES}
)


add_executable(example_gpu_pinhole src/example_gpu_pinhole.cpp)
target_link_libraries(example_gpu_pinhole
    ${imagine_LIBRARIES}
)

add_executable(example_gpu_o1dn src/example_gpu_o1dn.cpp)
target_link_libraries(example_gpu_o1dn
    ${imagine_LIBRARIES}
)