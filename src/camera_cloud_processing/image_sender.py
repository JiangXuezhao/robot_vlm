import os
import time
import base64

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import requests
import cv2
from cv_bridge import CvBridge

from aip import AipSpeech
import datetime

class ImageSender(Node):
    def __init__(self):
        super().__init__('image_sender')
        self.subscription = self.create_subscription(
            Image,
            'camera_image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.vlm_url = "https://api.siliconflow.cn/v1/chat/completions"
        # 创建保存图像的目录
        self.save_dir = 'received_images'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
        # 结果文本文件
        self.result_file = 'image_captions.txt'

    # rqt_graph 显示ros结构的
    # rviz2 显示可视化页面
    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        _, buffer = cv2.imencode('.jpg', cv_image)
        image_bytes = buffer.tobytes()

        # 保存图像到本地
        timestamp = str(int(time.time()))
        image_path = os.path.join(self.save_dir, f'image_{timestamp}.jpg')
        cv2.imwrite(image_path, cv_image)
        self.get_logger().info(f'Image saved to {image_path}')
        result = self.jpg_to_data_uri(image_path)

        if result:
            # print(result)
            self.get_logger().info("System is handling images...")

            # Qwen/Qwen2.5-VL-72B-Instruct
            # Pro/Qwen/Qwen2-VL-7B-Instruct
            payload = {
                "model": "Qwen/Qwen2.5-VL-72B-Instruct",
                "stream": False,
                "max_tokens": 512,
                "temperature": 0.7,
                "top_p": 0.7,
                "top_k": 50,
                "frequency_penalty": 0.5,
                "n": 1,
                "stop": [],
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {
                                "image_url": {
                                    "detail": "low",
                                    "url": result
                                },
                                "type": "image_url"
                            },
                            {"type": "text", "text": "图中描绘的是什么景象？"},
                        ]
                    }
                ]
            }
            headers = {
                "Authorization": "Bearer your_api_key",
                "Content-Type": "application/json"
            }

        try:
            start_time = time.time()
            response = requests.request("POST", self.vlm_url, json=payload, headers=headers)
            step1_time = time.time() - start_time

            if response.status_code == 200:
                get_result = response.json()
                self.get_logger().info(f"Received result from VLM: {get_result['choices'][0]['message']['content']}")
                # 这里可以将结果发布为 ROS 2 消息
                # 将结果和图片路径写入文本文件
                with open(self.result_file, 'a') as f:
                    f.write(f"{image_path}: {get_result['choices'][0]['message']['content']}\n")
                self.speak_text(str(get_result['choices'][0]['message']['content']))
            else:
                self.get_logger().error(f"Failed to send image to VLM. Status code: {response.status_code}")
        except requests.RequestException as e:
            self.get_logger().error(f"Request error: {e}")


    def speak_text(self, text):
        APP_ID = ''
        API_KEY = ""
        SECRET_KEY = ""

        # 初始化AipSpeech对象
        client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)

        # 调用合成接口
        result = client.synthesis(text, 'zh', 1, {
            'vol': 5,  # 音量，取值0-15，默认为5中音量
            'spd': 5,  # 语速，取值0-9，默认为5中语速
            'pit': 5,  # 音调，取值0-9，默认为5中语调
            'per': 0  # 发音人，0为普通女声，1为普通男声，3为度逍遥，4为度丫丫
        })

        now = datetime.datetime.now()
        formatted_time = now.strftime("%Y%m%d%H%M%S")
        audio_name = 'temp_audio{}.mp3'.format(formatted_time)

        # 识别正确返回语音二进制 错误则返回dict
        if not isinstance(result, dict):
            with open(audio_name, 'wb') as f:
                f.write(result)
            os.system('mpg321 {}'.format(audio_name))
            os.remove(audio_name)
        else:
            print(f"语音合成失败: {result}")

    def jpg_to_data_uri(self, image_path):
        try:
            with open(image_path, "rb") as image_file:
                image_data = image_file.read()
                base64_encoded = base64.b64encode(image_data).decode('utf-8')
                data_uri = f"data:image/jpeg;base64,{base64_encoded}"
                return data_uri
        except FileNotFoundError:
            print(f"文件未找到: {image_path}")
            return None
        except Exception as e:
            print(f"发生错误: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    image_sender = ImageSender()
    rclpy.spin(image_sender)
    image_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    