import tkinter as tk
from tkinter import ttk
from tkinter import filedialog as fd
from tkinter.messagebox import showinfo

from time import sleep
import sys
import os
  
current = os.path.dirname(os.path.realpath(__file__))
parent_directory = os.path.dirname(current)
sys.path.append(parent_directory)

from siyi_sdk import SIYISDK

# create the window window
window = tk.Tk()
window.title('SiYi Ground Control ( ͡❛ ͜ʖ ͡❛)')
# window.resizable(False, False)
window.geometry('320x240')


cam = SIYISDK(server_ip="192.168.144.25", port=37260)
if not cam.connect():
    print("No connection ")
    exit(1)


gimbalSpeed = 20
def pitch_up(event):
    cam.requestGimbalSpeed(0,gimbalSpeed)
    print("Attitude (yaw,pitch,roll) eg:", cam.getAttitude())
def pitch_down(event):
    cam.requestGimbalSpeed(0,-gimbalSpeed)
    print("Attitude (yaw,pitch,roll) eg:", cam.getAttitude())
def yaw_left(event):
    cam.requestGimbalSpeed(-gimbalSpeed,0)
    print("Attitude (yaw,pitch,roll) eg:", cam.getAttitude())
def yaw_right(event):
    cam.requestGimbalSpeed(gimbalSpeed,0)
    print("Attitude (yaw,pitch,roll) eg:", cam.getAttitude())
def gimbalStop(event):
    cam.requestGimbalSpeed(0,0)
    print("Attitude (yaw,pitch,roll) eg:", cam.getAttitude())

# pitch up button
pitch_up_button = ttk.Button(
    window,
    text='up',
)
pitch_up_button.bind("<ButtonPress-1>", pitch_up)
pitch_up_button.bind("<ButtonRelease-1>", gimbalStop)
pitch_up_button.grid(row = 0, column = 1, pady = 2)
# pitch down button
pitch_down_button = ttk.Button(
    window,
    text='down',
)
pitch_down_button.bind("<ButtonPress-1>", pitch_down)
pitch_down_button.bind("<ButtonRelease-1>", gimbalStop)
pitch_down_button.grid(row = 2, column = 1, pady = 2)
# yaw left button
yaw_left_button = ttk.Button(
    window,
    text='left',
)
yaw_left_button.bind("<ButtonPress-1>", yaw_left)
yaw_left_button.bind("<ButtonRelease-1>", gimbalStop)
yaw_left_button.grid(row = 1, column = 0, pady = 2)
# yaw right button
yaw_right_button = ttk.Button(
    window,
    text='right',
)
yaw_right_button.bind("<ButtonPress-1>", yaw_right)
yaw_right_button.bind("<ButtonRelease-1>", gimbalStop)
yaw_right_button.grid(row = 1, column = 2, pady = 2)

# 🎯

def picth_yaw_center():
    cam.requestCenterGimbal()
    print("Attitude (yaw,pitch,roll) eg:", cam.getAttitude())

# pitch down button
picth_yaw_center_button = ttk.Button(
    window,
    text='🎯',
    command=picth_yaw_center
)

picth_yaw_center_button.grid(row = 1, column = 1, pady = 2)
# picth_yaw_center_button.pack(expand=True)

# 🔎 ➕

def zoom_in():
    val = cam.requestZoomIn()
    sleep(0.5)
    val = cam.requestZoomHold()
    sleep(0.5)
    print("Zoom level: ", cam.getZoomLevel())

# pitch down button
zoom_in_button = ttk.Button(
    window,
    text='🔎➕',
    command=zoom_in
)

zoom_in_button.grid(row = 3, column = 0, pady = 2)
# zoom_in_button.pack(expand=True)


# 🔎 ➖ 

def zoom_out():
    val = cam.requestZoomOut()
    sleep(0.5)
    val = cam.requestZoomHold()
    sleep(0.5)
    print("Zoom level: ", cam.getZoomLevel())

# pitch down button
zoom_out_button = ttk.Button(
    window,
    text='🔎➖',
    command=zoom_out
)

zoom_out_button.grid(row = 3, column = 2, pady = 2)
# zoom_out_button.pack(expand=True)


import threading
from stream import SIYIRTSP

def streaming():
    rtsp = SIYIRTSP(rtsp_url="rtsp://192.168.144.25:8554/main.264", debug=False)
    rtsp.setShowWindow(True)





if __name__ == "__main__":
    streaming_thread = threading.Thread(target=streaming)
    streaming_thread.start()

    window.mainloop()
    
    streaming_thread.join()
    cam.disconnect()
    print("exit")


