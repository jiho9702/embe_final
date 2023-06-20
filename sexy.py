#!/usr/bin/python3
import RPi.GPIO as GPIO
import pigpio
import time
import cv2
import threading
import serial

from serial import *
from ultralytics import YOLO

cap = cv2.VideoCapture(0)

model = YOLO("best.pt")
# 전체 동물의 딕셔너리
detect_list = {0:'Bear', 1:'Bird', 2:'WaterDeer', 3:'Pig', 4:'Deer'}
objects = []
image_list = []

# 카메라의 위치를 저장하는 변수
position = 0

# 전체 동물의 검출 정보가 저장되는 배열
total = []

# 블루투스 연결부분 (포트 설정)
bluetooth_port = '/dev/rfcomm0'
baud_rate = 9600
bluetooth = serial.Serial(bluetooth_port, baud_rate)


# position = 카메라 번호, center = 객체가 있는 공간의 중심좌표, each = 각 동물의 라벨(이름)
def calc_degree(position, center, each):
  # d1 = 1번 카메라의 동물의 좌표, d2 = 2번 카메라의 동물의 좌표
  global d1, d2
  if position == 1:
      for i in range(len(center)):
        d1 = int(center[i] / 640 * 68.5 - 68.5)
        data = str(each[i]) + ',' + str(d1) + ','
        print(data)
        # 전체 배열에 포함해준다
        total.append(data)
        
        

  if position == 2:
      for i in range(len(center)):
        d2 = int(center[i]/ 640 * 68.5)
        data = str(each[i]) + ',' + str(d2) + ','
        print(data)
        # 전체 배열에 포함해준다
        total.append(data)
        

# 욜로 객체 인식 부분
def yolo(image_list):
  position = 1
  for image in image_list:
    # 각 동물 저장 배열
    each = []
    # 중심점 저장 배열
    center = []
    results = model(source=image, conf=0.7, show=False, device='cpu')
    for r in results:
        boxes = r.boxes  # Boxes object for bbox outputs
        masks = r.masks  # Masks object for segment masks outputs
        probs = r.probs  # Class probabilities for classification outputs
        objects.append(boxes.cls)
        # print("결과 = " + str(boxes.cls.tolist()))
        # 동물의 종류를 배열 화 해서 저장한다.
        items = boxes.cls.tolist()
        # if len(items) == 0:
        #     total.append(',0')
#			bluetooth.write(len(items))
        for item in items:
            print(detect_list[int(item)])
            data = detect_list[int(item)]
            each.append(data)
            #bluetooth.write(data.encode())
        # 객체 검출이 되는 바운딩 박스의 좌표를 저장 top_left x,y // bottom_right x,y
        coordi = boxes.xyxy.tolist()
#			bluetooth.write(len(coordi))
        for c in coordi:
            tl_x = int(c[0])
            tl_y = int(c[1])
            br_x = int(c[2])
            br_y = int(c[3])
            # global center_x
            center_x = (abs(tl_x + br_x)/2)
            center.append(center_x)
            center_y = abs(tl_y - br_y)
        calc_degree(position, center, each)
    position += 1

# 카메라 보여주는 코드 사실 필요없음
def detect(a):
  for i in range(20):
    a = cap.read()
  time.sleep(1)
  _, frame = cap.read()
  cv2.imshow("frame", frame)
  cv2.waitKey(1)
  return frame

# 서보모터를 돌려주는 코드
def servo_():
    servo = 14

    # more info at http://abyz.me.uk/rpi/pigpio/python.html#set_servo_pulsewidth

    pwm = pigpio.pi()
    pwm.set_mode(servo, pigpio.OUTPUT)

    pwm.set_PWM_frequency(servo, 50)

    cap = cv2.VideoCapture(0)

    while True:
        
        image_list = []

        # 왼쪽으로 회전
        pwm.set_servo_pulsewidth(servo, 1200)
        image1 = detect(3)
        image_list.append(image1)
        time.sleep(0.5)

        # print("90 deg")
        # pwm.set_servo_pulsewidth(servo, 1500)
        # time.sleep(3)
        
        # 오른쪽으로 회전
        pwm.set_servo_pulsewidth(servo, 1800)
        image2 = detect(3)
        image_list.append(image2)
        time.sleep(0.5)

        # 객체 검출로 보내는 코드
        yolo(image_list)

        # 전체 동물의 갯수
        animal_count = 0
        for t in total:
           #print("first = " + str(t[0]))
           if t[1] != '0':
              animal_count += 1
        # print(animal_count)
        if(animal_count == 0):
          cnt_data = str(animal_count) + ',;'
        else:
          cnt_data = str(animal_count) + ','
        

        bluetooth.write(cnt_data.encode())
        print("here is inside for")
        if len(total):
          for t in total:
            data = t
            print(t)
            bluetooth.write(data.encode())
          total.clear()
          endSymball = ";"
          
          bluetooth.write(endSymball.encode())
        else:
          print("nothing")
          
           
        



# turning off servo
# pwm.set_PWM_dutycycle(servo_, 0)
# pwm.set_PWM_frequency(servo_, 0)

# def main(): 
#   detect_thread = threading.Thread(target=detect, args=(None,))
#   servo_thread = threading.Thread(target=servo_, args=(None,))

#   detect_thread.start()
#   servo_thread.start()

#   detect_thread.join()
#   servo_thread.join()

# could not open port /dev/rfcomm0: [Errno 2] No such file or directory: '/dev/rfcomm0'
# sudo chmod 666 /dev/rfcomm0
# sudo pigpiod

if __name__ == "__main__":
  servo_()