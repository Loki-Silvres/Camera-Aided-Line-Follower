import cv2 as cv
import numpy as np
import time
import yaml
import serial
from pynput import keyboard
import threading

nano = serial.Serial("/dev/ttyUSB0", 2000000,timeout=0.1)

def show_video():
    with open(r"/home/pandoPi/py_codes/ost.yaml", 'r') as file:
        data = yaml.safe_load(file)
    camera_matrix = np.array(data['camera_matrix']['data']).reshape((3, 3))
    distortion_coefficients = np.array(data['distortion_coefficients']['data'])

    cap = cv.VideoCapture('/dev/video0')

    desired_resolution = (640, 480)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, desired_resolution[0])
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, desired_resolution[1])
    while 1:
        current_time = time.time()
        ret, frame = cap.read()
        frame = cv.undistort(frame, camera_matrix, distortion_coefficients)

        cv.imshow("cam", frame)        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        prev_time = time.time()
        print("loop_rate = ", -1/(current_time - prev_time))
        
    cv.destroyAllWindows()
    cap.release()

def on_press(key):
    try:
        pressed = key.char
        if(pressed in ['4','6','8','2']):
            msg = bytes(str(pressed), "utf-8")
            nano.write(msg)
        print('alphanumeric key {0} pressed'.format(
            key.char))
        
    except AttributeError:
        if key == keyboard.Key.enter:
            msg = bytes(str(5), "utf-8")
            nano.write(msg)
        print('special key {0} pressed'.format(
            key))
def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        return False

video_thread = threading.Thread(target=show_video, name='Video Thread')
keyboard_listener = keyboard.Listener(on_press=on_press, on_release=on_release)
video_thread.start()
keyboard_listener.start()
video_thread.join()
keyboard_listener.join()
