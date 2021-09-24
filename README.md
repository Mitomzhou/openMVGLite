## OpenMVG简化版
完整版在这里 https://github.com/openMVG/openMVG.git
### 一、环境安装 (Linux下)
#### 安装git
#### 安装 cmake
(建议源码安装，3.20版本免编译，直接复制到/usr/local/下，加入环境变量 
~~~bash
# cmake-3.20
export PATH=$PATH:/usr/local/cmake/bin
~~~
#### 安装依赖包
~~~bash
sudo apt-get install libpng-dev libjpeg-dev libtiff-dev libxxf86vm1 libxxf86vm-dev libxi-dev libxrandr-dev
sudo apt-get install graphviz
sudo apt-get install doxygen qtbase5-dev sphinxsearch
~~~
#### 安装openMVG
~~~bash
git clone https://github.com/openMVG/openMVG.git
cd openMVG
mkdir build
cd build
cmake ..
make -j4
sudo make install
~~~
#### 安装ceres
下载 http://ceres-solver.org/ceres-solver-1.13.0.tar.gz
~~~bash
sudo apt-get install cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
cd ceres-solver-1.13.0
mkdir build & cd build
cmake ..
make -j10
sudo make install
~~~
#### 安装OpenCV （略）
