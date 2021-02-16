## Prerequisition
### Eigen installation

### yaml-cpp
release-0.5.3
```
git clone https://github.com/jbeder/yaml-cpp
cd yaml-cpp
mkdir build
cd build
cmake .. -DYAML_BUILD_SHARED_LIBS=ON
make -j4
sudo make install
sudo cp libyaml-cpp.so.0.6 /usr/lib/
```