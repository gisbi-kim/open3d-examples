# open3d-examples
This repository of non-official Opend3D examples covers 3D point cloud processing acquired from outdoor.



## Dependancies 

Open3D
```
git clone https://github.com/IntelVCL/Open3D
util/scripts/install-deps-ubuntu.sh
mkdir build
cd build
cmake -DBUILD_EIGEN3=ON  \
      -DBUILD_GLEW=ON    \
      -DBUILD_GLFW=ON    \
      -DBUILD_JPEG=ON    \
      -DBUILD_JSONCPP=ON \
      -DBUILD_PNG=ON     \
      ..
make -j
make install
```


## Sample data 

Freiburg Campus 360 degree 3D scans

```
http://ais.informatik.uni-freiburg.de/projects/datasets/fr360/
```
