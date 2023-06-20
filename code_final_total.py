import threading
import time
import RPi.GPIO as GPIO
from ultralytics import YOLO
import cv2
import numpy as np
import serial
from serial import *

position = 0

def calc_degree(position, center_x):
    
    if position == 1:
        d = int(center_x) / 640 * 68.5 - 68.5
        return d

    if position == 2:
        d = int(center_x) / 640 * 68.5
        return d
    

def servo(a):
    # from servo_big import send_pos
    # position = send_pos(1)
    # print(position)

    servoPIN = 14
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(servoPIN, GPIO.OUT)
    p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
    p.start(2.5) # Initialization

    p.ChangeDutyCycle(7.5)

    pos = 0
    global position
    # try:
    while True:
        #7.5 standard
        # a = input()
        # a = float(a)
        # p.ChangeDutyCycle(a)

    # 3.3이 왼쪽 90도
    # 5.4이 왼쪽 45도
    # 7.5 중심 (4.2씩 차이)
    # 9.6이 오른쪽 45도
    # 11.7이 오른쪽 90도
      
        
        p.ChangeDutyCycle(5.95)     # 왼쪽
        position = 1   
        time.sleep(5)   
            
    
        p.ChangeDutyCycle(9.05)     # 오른쪽
        position = 2
        time.sleep(5)

    # except KeyboardInterrupt:
    #   p.stop()
    #   GPIO.cleanup()

def detect(model):
    # from detect import detect
    # center = detect()
    # print(position)
    # print(center)
    # d = calc_degree(position, center)
    # # add code servo

    #bluetooth_port = '/dev/rfcomm0'
    #baud_rate = 9600
    #bluetooth = serial.Serial(bluetooth_port, baud_rate)

    cap = cv2.VideoCapture(0)

    detect_list = {0:'Bear', 1:'Bird', 2:'WaterDeer', 3:'Pig', 4:'Deer'}

    objects = []

    while cap.isOpened():
        for i in range(10):
            a = cap.read()
        ret, frame = cap.read()
        results = model(source=frame, conf=0.7, show=False, save_txt=True, device='mps')
        # print(results.pred)
        # print(results)
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
                print(d)
                 
    # while cap.isOpened():
    # 	ret, frame = cap.read()
    # 	result = model.predict(source=frame, show=True, device="cpu")


    # 	bluetooth.write(data.encode())ㄴ

    cv2.destroyAllWindows()
    #bluetooth.close()


def main():
    model = YOLO("best.pt")

    detect_thread = threading.Thread(target=detect, args=(model,))
    servo_thread = threading.Thread(target=servo, args=(None,))

    detect_thread.start()
    servo_thread.start()

    # detect_thread.s()
    # servo_thread.start()

    detect_thread.join()
    servo_thread.join()

if __name__ == "__main__":
    main()