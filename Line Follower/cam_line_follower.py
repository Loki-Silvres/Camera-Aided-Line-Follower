import cv2 as cv
import numpy as np 
import time
import yaml
cfrom pynput import keyboard
import threading

nano = serial.Serial("/dev/ttyUSB0", 2000000,timeout=0.1)
IMG_PATH="Photos and Videos/track_images"
vel = 42
Turn_Vel = 45
turn_vel = 40
Kp = 6e-7
interval = 2 # seconds to decide off-track
current_time = time.time()
prev_time = time.time()


flag_manual = False

def sendManualCommand(command):
    global nano, flag_manual
    if not flag_manual:
        return
    msg = "MAN" + str(command) 
    print(msg)
    msg = bytes(msg, "utf-8")
    nano.write(msg)

def sendReverse(turn_vel):
    global nano, flag_manual
    if flag_manual:
        return
    msg = "REV" + str(turn_vel)
    print(msg)
    msg = bytes(msg, "utf-8")
    nano.write(msg)

def sendSpeed(leftSpeed, rightSpeed):
    global nano, flag_manual
    if flag_manual:
        return
    msg = "VL" + str(leftSpeed) + "VR" + str(rightSpeed)
    print(msg)
    msg = bytes(msg, "utf-8")
    nano.write(msg)

def follow_line(frame):
    global current_time, prev_time, interval, turn_vel
    h, w, _ = frame.shape
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    blur = cv.GaussianBlur(gray, (3,3), 0)
    _, thres = cv.threshold(blur, 50, 255, cv.THRESH_BINARY_INV)
    n_slits = 4
    step = h//n_slits

    setpoint = w//2
    angle = 0
    cX_s = []

    for i in range(n_slits):
        mask = thres[i*step:(i+1)*step]
        result = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnt, h = result[1], result[0]
        try:
            contour = max(cnt, key=cv.contourArea)
            # print(cv.contourArea(contour))
            
            if cv.contourArea(contour)>1000:
                mask = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
                crack = np.zeros_like(mask)
                M = cv.moments(contour)
                cX = int(M["m10"] / M["m00"])
                cX_s.append(cX)
                cY = int(M["m01"] / M["m00"])
                # mask = cv.drawContours(crack, contours=[contour], contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv.LINE_AA)
                # cv.circle(mask, (cX, cY), 7, (255, 255, 255), -1)
                # cv.putText(mask, "center", (cX - 20, cY - 20),
                # cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

##                cv.imshow("cam", thres) 
##                if cv.waitKey(1) & 0xFF == ord('q'):
##                    return
                current_time = time.time()
                prev_time = time.time()
                
            else:
                current_time = time.time()
                if(current_time-prev_time>interval):
                    sendReverse(turn_vel)
                
        except:
            pass
    if len(cX_s):
        weights = np.linspace(1,0.5,len(cX_s))
        weights /= np.sum(weights)
        avg_cX = np.sum(np.multiply(cX_s, weights))
        setpoint = avg_cX - w//2
        if setpoint > 0:
            turn_vel = Turn_Vel
        else:
            turn_vel = -Turn_Vel
        try:
            angle = cX_s[-2] - cX_s[-1]
            angle /= step
            angle = np.arctan(angle)*180/np.pi
        except:
            angle = cX_s[-1] - w//2
        
        # print(cX_s, weights)
        # print("setpoint: ",setpoint, "width: ", w)
        return setpoint, angle
    

def show_video():
    global vel, Kp
    itr = 0
    with open(r"config/ost.yaml", 'r') as file:
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
##        frame = cv.imread(IMG_PATH+str(itr)+'.png')
        frame = cv.undistort(frame, camera_matrix, distortion_coefficients)
        
        result = follow_line(frame)
        if(result!=None):
            setpoint, angle = result
##            print(setpoint, angle)
            error = setpoint ** 3
            P = Kp * error
            leftSpeed = int(vel + P)
            rightSpeed = int(-(vel - P)) 
            sendSpeed(leftSpeed=leftSpeed, rightSpeed=rightSpeed)
        itr+=1
        if(itr>=15):
            itr = 0
        
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (3,3), 0)
        _, thres = cv.threshold(blur, 50, 255, cv.THRESH_BINARY_INV)
        cv.imshow("cam", thres)        

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        prev_time = time.time()
        print("loop_rate = ", -1/(current_time - prev_time))
        
    cv.destroyAllWindows()
    # cap.release()

def on_press(key):
    global flag_manual
    try:
        pressed = key.char
        if(pressed in ['4','6','8','2','3','9']):
            sendManualCommand(pressed)
        print('alphanumeric key {0} pressed'.format(
            key.char))
        if pressed == 'q' or pressed == 'Q':                            # Press 'q' to stop video
            sendSpeed(0,0)
##            return False
        
    except AttributeError:
        if key == keyboard.Key.enter:
            flag_manual = not flag_manual
            sendManualCommand('5')
        print('special key {0} pressed'.format(
            key))
def on_release(key):
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:                                         # Press 'esc' to stop motors ans program
        sendSpeed(0,0)
        return False

video_thread = threading.Thread(target=show_video, name='Video Thread')
keyboard_listener = keyboard.Listener(on_press=on_press, on_release=on_release)
video_thread.start()
keyboard_listener.start()
video_thread.join()
keyboard_listener.join()
