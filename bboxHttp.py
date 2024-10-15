import threading
from flask_cors import CORS
from flask import Flask, render_template, Response, request
import cv2
import os
import sys
import rclpy
from rclpy.node import Node
from tutorial_interfaces.msg import Bbox

# 设置摄像头RTSP链接
cam = 'rtsp://169.254.220.160:8554/main.264'
app = Flask(__name__)
CORS(app)

# 初始化ROS节点
class BboxSubscriber(Node):
    def __init__(self):
        super().__init__("bbox_subscriber")
        self.bbox = [0, 0, 0, 0]  # 初始化bbox
        self.create_subscription(Bbox, "bbox", self.bbox_callback, 5)

    def bbox_callback(self, msg):
        self.bbox = [msg.x0, msg.y0, msg.x1, msg.y1]  # 更新bbox

def init_ros():
    rclpy.init()
    return BboxSubscriber()

camera = cv2.VideoCapture(cam)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 5000)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 5000)

# 读取摄像头帧
def readcam():
    while True:
        global frame
        success, frame = camera.read()
        if not success:
            break

# 生成视频帧，叠加bbox
def gen_frames():
    global frame
    ros_node = init_ros()
    while True:
        rclpy.spin_once(ros_node)  # 处理ROS回调

        if frame is not None:
            # 获取bbox
            x0, y0, x1, y1 = ros_node.bbox
            
            # 在帧上绘制bbox
            cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 0, 0), 2)

            _, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed', methods=["GET"])
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    threading.Thread(target=readcam).start()
    threading.Thread(target=lambda: rclpy.spin(init_ros())).start()  # 启动ROS
    app.run('0.0.0.0', threaded=True, port=5001)

