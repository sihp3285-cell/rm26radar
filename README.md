# 简介
> ultralytics的TensorRT10 C++实现

# 环境配置
- opencv 4.11
- cuda 12.5
- cudnn 9.8
- TensorRT 10.11.0.33
- yaml-cpp 0.8.0

## OPENCV
``` sh
mkdir opencv_build && cd opencv_build
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DOPENCV_GENERATE_PKGCONFIG=ON -DBUILD_opencv_legacy=OFF -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH= ../opencv_contrib/modules/ ../opencv

make -j8
sudo make install
```

**默认已经完成nvidia显卡驱动的安装，若未安装，请先安装，并使用`nvidia-smi`检查驱动是否安装成功，并观察支持的cuda版本**

## YAML-CPP
``` sh
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp && mkdir build && cd build
cmake ..
make -j8
sudo make install
```

## CUDA
[CUDA Toolkit Archive | NVIDIA Developer](https://developer.nvidia.com/cuda-toolkit-archive)
在官网寻找适合的cuda版本进行安装

安装完成后，在`/usr/local`目录下，应该能看到`cuda`的目录。此时可以使用`nvcc -V`，观察是否能正常显示版本信息，若正常显示版本信息，则直接进行`cudnn`的安装。

**终端运行**
``` sh
sudo touch /etc/profile.d/cuda.sh
echo 'export PATH=/usr/local/cuda/bin/:$PATH' | sudo tee -a /etc/profile.d/cuda.sh
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64/:/usr/lib/wsl/lib/:$LD_LIBRARY_PATH' | sudo tee -a /etc/profile.d/cuda.sh
```

**编辑`~/.bsahrc`添加下面内容，编辑完成后，记得执行`source ~/.bashrc`**
``` sh
# 下面路径请自行确认，一般只有cuda版本需要修改
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export PATH=/usr/local/cuda-12.5/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.5/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export CUDA_HOME=/usr/local/cuda-12.5
```

完成后再次尝试`nvcc -V`观察是否正常显示`cuda`版本

## CUDNN
[cuDNN Archive | NVIDIA Developer](https://developer.nvidia.com/cudnn-archive)
按照安装的`cuda`版本，选择`cudnn`版本进行安装，其中`cudnn`也区分8，9两个大版本

**8版本的`cudnn`可以使用下载压缩包的形式，通过解压，移动其中的文件进行配置**
``` sh
tar -xvf cudnn-linux-x86_64-8.9.7.29_cuda12-archive.tar.xz
sudo cp cudnn-*-archive/include/cudnn*.h /usr/local/cuda/include
sudo cp -P cudnn-*-archive/lib/libcudnn* /usr/local/cuda/lib64 
sudo chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn*
```
**也可以下载`deb`文件进行二进制安装**

**9版本的`cudnn`则使用配置apt源的二进制方式安装**

## TensorRT
[TensorRT Download | NVIDIA Developer](https://developer.nvidia.com/tensorrt/download)
寻找合适版本的`TensorRT`进行安装(由于`TensoRT`10版本对`API`有较多修改，所以运行本仓库代码则肯定是使用10版本的)

**这里通过下载`tar`文件进行配置**
``` sh
tar -zxvf TensorRT-10.11.0.33.Linux.x86_64-gnu.cuda-12.9.tar.gz
```

解压后，在`~/.bashrc`添加下面内容，注意路径
``` sh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/lin/TensorRT-10.11.0.33/lib
```
进入`python`目录，可以下载对应`TensorRT`版本的`python包`

### 测试
``` sh
cd samples/sampleOnnxMNIST/
make
# 切换到TensorRT-{版本}/targets/x86_64-linux-gnu/bin目录下运行sample_onnx_mnist
cd ../../targets/x86_64-linux-gnu/bin/
./sample_onnx_mnist
```
**若编译与运行都正常，则说明安装成功**

# 部署
当环境配置正常，则该仓库代码部署也应该没什么问题.
一般`git`下来后，修改`CMakeLists.txt`中`TensorRT`的路径，以及`main.cpp`中的文件路径后，进行编译即可。

``` sh
git clone https://github.com/LZY-XiXi/tensorrt10_detect.git
cd tensorrt10_detect
mkdir build && cd build
cmake ..
make -j8
./standard
```

# 另说
本仓库代码是针对`ultralytics`的，所以`engine`文件需要利用`ultralytics`仓库代码进行生成。其中，直接使用`ultralytics`仓库的`export`是会出现模型文件序列化失败的。
[原因，解决方案](https://blog.csdn.net/ogebgvictor/article/details/145858668)

另外，如果手动使用`trtexec`转化`onnx`文件，路径请不要使用`~`符号，会出现无法识别而生成`engine`文件失败的。
