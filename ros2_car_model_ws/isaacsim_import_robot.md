
# IsaacSim控制ranger_with_xarm6
## 1.安装Isaac_sim环境
### 创建Conda环境
```bash
conda create -n env_isaaclab python=3.10
conda activate env_isaaclab
```
### 安装PyTorch
```bash
pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu129
```
### 安装isaacsim
``` bash
pip install 'isaacsim[all,extscache]==4.5.0' --extra-index-url https://pypi.nvidia.com
```
在安装了Isaacsim的conda环境下输入isaacsim即可启动gui界面
## 2.加载机器人模型
### 构建ros2工作空间
解压ros2_car_model_ws.zip得到ros2_car_model_ws文件夹，打开终端进入该文件夹下，输入命令`colcon build`编译环境。

打开工作空间下scripts文件夹内的load_keyboard_control.py文件，修改urdf_path内容，将/home/gm_2774373792/ros2_car_model/ros2_car_model_ws/替换为自己工作空间的路径。vel为小车速度，angel为转向角度。

打开终端，在刚刚安装了isaacsim的conda环境下运行`python ${ros2_car_model_ws}/scripts/load_keyboard_control.py`即可一键加载模型并初始化关节控制句柄。点击gui界面即可使用WASD控制小车运动。

