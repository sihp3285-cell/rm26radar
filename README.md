# 简介
> Radar27：ROS 2 雷达感知、定位、跟踪、先验与业务输出。

包边界、消息契约、资源迁移与完整构建说明见 [ARCHITECTURE.md](ARCHITECTURE.md)。

# 环境配置
- opencv 4.11
- cuda 12.5
- cudnn 9.8
- TensorRT 10.11.0.33
- yaml-cpp 0.8.0

## OPENCV
相机内录使用 OpenCV 的 GStreamer 后端，编译 OpenCV 前安装开发库与录像插件：

```sh
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
```

``` sh
mkdir opencv_build && cd opencv_build
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DWITH_GSTREAMER=ON -DOPENCV_GENERATE_PKGCONFIG=ON -DBUILD_opencv_legacy=OFF -DCMAKE_INSTALL_PREFIX=/usr/local -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ../opencv

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

# 部署（ROS2 主链）

总入口为 `radar27_bringup`。以下命令使用已确认存在的本机路径：

- 工作区：`/home/delphine/rm/radar27`
- 模型目录：`/home/delphine/rm/radar27/models`（`robot_only.engine`、`newarmor.engine`、`classify_hku.engine`）
- 回放视频：`/home/delphine/rm/car_project/test/08.mp4`
- 运行目录：`/home/delphine/.local/state/radar27`

## 每个新终端先加载环境

```bash
cd /home/delphine/rm/radar27
source /opt/ros/jazzy/setup.bash
source /home/delphine/rm/radar27/install/setup.bash
```

修改代码后先重新构建再启动；本机已有构建目录可使用：

```bash
cd /home/delphine/rm/radar27
colcon build --base-paths /home/delphine/rm/radar27/src --cmake-args -DBUILD_CAMERA=ON
source /home/delphine/rm/radar27/install/setup.bash
```

本机 MVS SDK 位于 `/opt/MVS`，现有构建已启用 `BUILD_CAMERA=ON`。
相机默认使用海康、序列号 `DA7831910`，配置位于
`/home/delphine/rm/radar27/src/radar27_detection/config/params.yaml`。
首次安装的完整依赖与构建说明见 [ARCHITECTURE.md](ARCHITECTURE.md#构建与启动)。

以下各模式任选一个启动。默认开启 Qt 显示和标定/ROI 工具，默认己方蓝色；
切换己方红色可在命令末尾加 `own_team:=red`。退出使用 Ctrl+C。

## 视频实时回放

按视频帧率播放，检测跟不上时允许覆盖待处理帧；播放结束自动退出。

```bash
ros2 launch radar27_bringup detect_pipeline.launch.py \
  mode:=video \
  video_path:=/home/delphine/rm/car_project/test/08.mp4 \
  model_dir:=/home/delphine/rm/radar27/models \
  runtime_dir:=/home/delphine/.local/state/radar27 \
  playback_mode:=realtime \
  enable_recording:=false
```

## 视频逐帧回放

等待检测消费后再提交下一帧，适合逐帧分析；运行速度由处理能力决定，播放结束自动退出。

```bash
ros2 launch radar27_bringup detect_pipeline.launch.py \
  mode:=video \
  video_path:=/home/delphine/rm/car_project/test/08.mp4 \
  model_dir:=/home/delphine/rm/radar27/models \
  runtime_dir:=/home/delphine/.local/state/radar27 \
  playback_mode:=sequential \
  enable_recording:=false
```

## 视频回放并录制

录制输入原图，沿用 `recording.codec`，默认 MP4V。录像队列独立于检测，队列满时仍会丢帧；逐帧回放也不保证录像无丢帧。

```bash
ros2 launch radar27_bringup detect_pipeline.launch.py \
  mode:=video \
  video_path:=/home/delphine/rm/car_project/test/08.mp4 \
  model_dir:=/home/delphine/rm/radar27/models \
  runtime_dir:=/home/delphine/.local/state/radar27 \
  playback_mode:=realtime \
  enable_recording:=true
```

## 相机实时检测

读取工业相机，不保存录像。

```bash
ros2 launch radar27_bringup detect_pipeline.launch.py \
  mode:=camera \
  model_dir:=/home/delphine/rm/radar27/models \
  runtime_dir:=/home/delphine/.local/state/radar27 \
  enable_recording:=false
```

## 相机实时检测并内录（GStreamer）

通过 GStreamer 将相机原图编码为 H.264/MP4，需要上文列出的 GStreamer 依赖与 OpenCV 后端支持。

```bash
ros2 launch radar27_bringup detect_pipeline.launch.py \
  mode:=camera \
  model_dir:=/home/delphine/rm/radar27/models \
  runtime_dir:=/home/delphine/.local/state/radar27 \
  enable_recording:=true
```

## 无界面视频回放

关闭 Qt 显示、标定/ROI 工具和 RViz，继续输出检测、定位与决策话题。

```bash
ros2 launch radar27_bringup detect_pipeline.launch.py \
  mode:=video \
  video_path:=/home/delphine/rm/car_project/test/08.mp4 \
  model_dir:=/home/delphine/rm/radar27/models \
  runtime_dir:=/home/delphine/.local/state/radar27 \
  playback_mode:=realtime \
  enable_recording:=false \
  enable_qt_display:=false enable_tools:=false enable_rviz:=false
```

## 无界面相机检测并内录

相机持续采集并保存原图录像，不启动可视化和交互工具。

```bash
ros2 launch radar27_bringup detect_pipeline.launch.py \
  mode:=camera \
  model_dir:=/home/delphine/rm/radar27/models \
  runtime_dir:=/home/delphine/.local/state/radar27 \
  enable_recording:=true \
  enable_qt_display:=false enable_tools:=false enable_rviz:=false
```

## RViz 调试

回放本机视频并开启 RViz 调试显示；相机调试时将 `mode:=video` 改为 `mode:=camera` 并删除 `video_path` 和 `playback_mode` 两项。

```bash
ros2 launch radar27_bringup detect_pipeline.launch.py \
  mode:=video \
  video_path:=/home/delphine/rm/car_project/test/08.mp4 \
  model_dir:=/home/delphine/rm/radar27/models \
  runtime_dir:=/home/delphine/.local/state/radar27 \
  playback_mode:=realtime \
  enable_recording:=false \
  rviz_debug_enabled:=true enable_rviz:=true
```

## 内录参数与输出

相机录像保存至 `/home/delphine/.local/state/radar27/recordings/camera_<时间戳>.mp4`，
视频模式录像保存至同目录下的 `video_<时间戳>.mp4`。采集帧在检测抽帧前进入独立录像队列，相机模式的 GStreamer 使用
`appsrc → videoconvert → x264enc → h264parse → mp4mux → filesink` 编码 H.264/MP4。
队列满时丢弃最旧待录帧；正常退出排空队列并封装文件，请用 Ctrl+C 停止。
`radar27_detection/config/params.yaml` 中的 `recording.encoder` 可指定接受 I420 输入的
GStreamer H.264 编码器及其参数，`recording.queue_size` 控制队列上限；
`recording.fps` 为文件播放帧率，不控制相机采集频率，应与采集帧率匹配，丢帧时录像时长会缩短。
视频模式仍使用 `recording.codec`。OpenCV 未启用 GStreamer 时开启相机内录会明确报错。
管线组件参考 [x264enc](https://gstreamer.freedesktop.org/documentation/x264/index.html)
与 [mp4mux](https://gstreamer.freedesktop.org/documentation/isomp4/mp4mux.html)。

无界面运行加 `enable_qt_display:=false enable_tools:=false`，结构化输出继续由 `radar27_decision` 提供。
RViz 说明见 [RVIZ_DEBUG.md](src/radar27_visualization/RVIZ_DEBUG.md)。

# 另说
本仓库代码是针对`ultralytics`的，所以`engine`文件需要利用`ultralytics`仓库代码进行生成。其中，直接使用`ultralytics`仓库的`export`是会出现模型文件序列化失败的。
[原因，解决方案](https://blog.csdn.net/ogebgvictor/article/details/145858668)

另外，如果手动使用`trtexec`转化`onnx`文件，路径请不要使用`~`符号，会出现无法识别而生成`engine`文件失败的。

## 世界坐标方向性误差

`pose_node` 会在每个检测框底边中心周围批量发射 5 条射线，以局部
`pixel -> world(x,z)` 雅可比传播像素误差，得到每帧方向相关的二维测量协方差。
远场视线方向会自动获得更低的 Kalman 增益；相邻射线命中沟底和高台等不同
PLY 高度面时会进一步放大该帧不确定性。相关参数位于
`src/radar27_localization/config/params.yaml` 的 `pose_node` 段。

## 测试与架构检查

检查与测试命令见 [ARCHITECTURE.md](ARCHITECTURE.md#验证)。
现有融合/先验回归测试保留，新增跨进程接口检查和包依赖边界检查。
