import threading
import time
import RPi.GPIO as GPIO
from ultralytics import YOLO
import cv2
import numpy as np
import serial
from serial import *

position = 0

model = YOLO('best.pt')

servoPIN = 14
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(2.5) # Initialization

p.ChangeDutyCycle(7.5)

pos = 0
d = 0

cap = cv2.VideoCapture(0)

detect_list = {0:'Bear', 1:'Bird', 2:'WaterDeer', 3:'Pig', 4:'Deer'}
objects = []
image_list = []


def calc_degree(position, center_x):
    if position == 1:
        d = int(center_x) / 640 * 68.5 - 68.5
        return d

    if position == 2:
        d = int(center_x) / 640 * 68.5
        return d

def detect(iamge_list):
    for image in image_list:
        results = model(source=image, conf=0.7, show=False, save_txt=True, device='cpu')
        for r in results:
            boxes = r.boxes  # Boxes object for bbox outputs
            masks = r.masks  # Masks object for segment masks outputs
            probs = r.probs  # Class probabilities for classification outputs
            objects.append(boxes.cls)
            # print("결과 = " + str(boxes.cls.tolist()))
            items = boxes.cls.tolist()
#			bluetooth.write(len(items))
            for item in items:
                print(detect_list[int(item)])
                data = 'd' + detect_list[int(item)] + '\n'
                #bluetooth.write(data.encode())
            coordi = boxes.xyxy.tolist()
#			bluetooth.write(len(coordi))
            for c in coordi:
                tl_x = int(c[0])
                tl_y = int(c[1])
                br_x = int(c[2])
                br_y = int(c[3])
                # global center_x
                center_x = (abs(tl_x - br_x))
                center_y = abs(tl_y - br_y)
                print(position)
                print(center_x)
                d = calc_degree(int(position), int(center_x))


try:
    while True:
        # 3.3이 왼쪽 90도
        # 5.4이 왼쪽 45도
        # 7.5 중심 (4.2씩 차이)
        # 9.6이 오른쪽 45도
        # 11.7이 오른쪽 90도
        
        p.ChangeDutyCycle(5.95)     # 왼쪽
        position = 1
        ret, frame1 = cap.read()
        
        p.ChangeDutyCycle(9.05)     # 오른쪽
        position = 2
        ret, frame2 = cap.read()

        image_list.append(frame1)
        image_list.append(frame2)

        detect(image_list)

        print(d)


except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()