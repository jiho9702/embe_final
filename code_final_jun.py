import pigpio
import time
import cv2


pi = pigpio.pi()
servo_pin = 14


def angle_to_pulsewidth(angle):
    pulsewidth = (angle * 2000 / 180) + 500
    return int(pulsewidth)


def set_servo_angle(pin, angle):
    pulsewidth = angle_to_pulsewidth(angle)
    pi.set_servo_pulsewidth(pin, pulsewidth)


set_servo_angle(servo_pin, 0)
time.sleep(1)

cap = cv2.VideoCapture(0)


try:
    while True:
        ret, frame = cap.read()
        cv2.imshow("image", frame)
        if cv2.waitKey(1) == ord('q'):
            break
        cap.release()

        # 0도에서 180도로 움직이기
        for angle in range(0, 180, 90):
            set_servo_angle(servo_pin, angle)
            time.sleep(1)

        # 180도에서 0도로 움직이기
        for angle in range(180, 0, -90):
            set_servo_angle(servo_pin, angle)
            time.sleep(1)

except KeyboardInterrupt:
    pi.set_servo_pulsewidth(servo_pin, 0)
    pi.stop()
