from ultralytics import YOLO
import cv2
import numpy as np
import serial
from serial import *

#bluetooth_port = '/dev/rfcomm0'
#baud_rate = 9600
#bluetooth = serial.Serial(bluetooth_port, baud_rate)

model = YOLO("best.pt")

cap = cv2.VideoCapture(0)

detect_list = {0:'Bear', 1:'Bird', 2:'WaterDeer', 3:'Pig', 4:'Deer'}

objects = []

while cap.isOpened():
	ret, frame = cap.read()
	results = model(source=frame, conf=0.7, show=False, save_txt=True, device='cpu')
	# print(results.pred)
	print(results)
	for r in results:
		print("in for")
		boxes = r.boxes  # Boxes object for bbox outputs
		masks = r.masks  # Masks object for segment masks outputs
		probs = r.probs  # Class probabilities for classification outputs
		objects.append(boxes.cls)
		print("결과 = " + str(boxes.cls.tolist()))
		items = boxes.cls.tolist()
		print(len(items))
#		bluetooth.write(len(items))
		for item in items:
			print(detect_list[int(item)])
			data = 'd' + detect_list[int(item)] + '\n'
			print(data)
			#bluetooth.write(data.encode())
		coordi = boxes.xyxy.tolist()
#			bluetooth.write(len(coordi))
		for c in coordi:
			tl_x = int(c[0])
			tl_y = int(c[1])
			br_x = int(c[2])
			br_y = int(c[3])
			center_x = 'c' + str(abs(tl_x - br_x))
			print(center_x)
			center_y = abs(tl_y - br_y)
#				bluetooth.write(center_x.encode())
				
				
			

# while cap.isOpened():
# 	ret, frame = cap.read()
# 	result = model.predict(source=frame, show=True, device="cpu")


# 	bluetooth.write(data.encode())

cv2.destroyAllWindows()
#bluetooth.close()
