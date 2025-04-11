# 运行方法
### 1.本项目是运行在ROS2平台下的图像识别+语音生成系统。
### 2. 安装requirements.txt中项目所需要的库
pip install -r requirements.txt

### 3.本项目vlm为Qwen/Qwen2.5-VL-72B-Instruct,
用户可根据需要选择自己想用的模型，接口是基于硅基流动平台的，所以需要
在这个平台内获取到对应的api key

### 4.本项目的文本转语音使用百度智能语音接口，用户可在官方文档中获取对应的
APP_ID、API_KEY、SECRET_KEY

# 使用方法
### 1.在linux系统下，开启2个终端窗口

### 2.cd ~\robot_vlm\src\camera_cloud_processing

### 3.Terminal 1 先执行 python camera_publisher.py

### 4.Terminal 2 再执行 python image_sender.py

