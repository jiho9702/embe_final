import RPi.GPIO as GPIO
import pigpio
import time
import cv2
import threading

servo = 14

# more info at http://abyz.me.uk/rpi/pigpio/python.html#set_servo_pulsewidth

pwm = pigpio.pi()
pwm.set_mode(servo, pigpio.OUTPUT)
pwm.set_PWM_frequency(servo, 50)


def detect(a):
  cap = cv2.VideoCapture(0)

  while True: 
    _, frame = cap.read()
    cv2.imshow("image", frame)
    cv2.waitKey(1)


def servo(a):
  while True:
    print("0 deg")
    pwm.set_servo_pulsewidth(servo, 1312)
    time.sleep(3)

    # print("90 deg")
    # pwm.set_servo_pulsewidth(servo, 1500)
    # time.sleep(3)

    print("180 deg")
    pwm.set_servo_pulsewidth(servo, 1912)
    time.sleep(3)

def main(): 
  detect_thread = threading.Thread(target=detect, args=(None,))
  servo_thread = threading.Thread(target=servo, args=(None,))

  detect_thread.start()
  servo_thread.start()

  detect_thread.join()
  servo_thread.join()

if __name__ == "__main__":
  main()