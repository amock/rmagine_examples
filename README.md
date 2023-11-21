# rmagine_examples

Examples that showcase the functionalities of the [rmagine](https://github.com/uos/rmagine) library.


## Run all

```console
mkdir build
cd build 
cmake ..
make
```



## CMakeLists.txt

The CMakeLists.txt shows how to find rmagine on your system and link its components to your executables.


## Samples

X = Computing device: cpu | gpu

### example_X_sphere

```console
./example_X_sphere ../dat/sphere.ply
```

Simulates a spherical sensor model plus gaussian noise and stores the results in a XYZ file:

![points_X_sphere.xyz](media/points_sphere.png)

### example_X_pinhole

```console
./example_X_pinhole ../dat/sphere.ply
```

Simulates a pinhole sensor model and stores the results in a XYZ file:

![points_X_pinhole.xyz](media/points_pinhole.png)

### example_X_o1dn

Simulates a sensor model with one origin and N customized directions. The results are stored in a XYZ file: 

![points_X_o1dn.xyz](media/points_o1dn.png)

### example_X_ondn

Simulates a sensor model with both custom origins and directions. The results are stored in a XYZ file:

![points_X_ondn.xyz](media/points_ondn.png)

### example_X_change_map

Demonstrates how to change maps or map elements on the fly. Results for two different maps are saved to XYZ files:


| map 1 (sphere) | map 2 (cube) |
|:----:|:----:|
|  ![points_cm_sphere.xyz](media/points_cm_sphere.png) |   ![point_cm_cube.xyz](media/points_cm_cube.png)   |


(WARNING: OptiX version shows a double free corruption error. TODO: check this)


