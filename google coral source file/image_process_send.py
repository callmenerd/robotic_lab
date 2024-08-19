import cv2
import threading
import time
from ultralytics import YOLO
import zmq
import subprocess
import json


def find_cam(name):
    output = subprocess.run(["./cam_with_name", name], capture_output=True)
    dev_name = str(output.stdout)

    def find_first_number(text):
        for char in text:
            if char.isdigit():
                return char
        return None

    return find_first_number(dev_name)

CAMERA_BALL = int(find_cam("Pro")) # BALL
CAMERA_SILO = int(find_cam("NYK")) # SILO

print(f"ball: {CAMERA_BALL} silo: {CAMERA_SILO}")

# Load YOLOv8 model
model = YOLO('best_full_integer_quant_edgetpu.tflite', task="detect", verbose=False)  # Replace with your model path if needed
# model.conf = 0.7

# Global variables for threading
frame1 = None
frame2 = None
lock = threading.Lock()
stop_event = threading.Event()
current_camera = CAMERA_BALL  # 0 for camera 1, 1 for camera 2
prev_cam = CAMERA_BALL
context = zmq.Context()

pub = context.socket(zmq.PUB)
pub.bind('tcp://127.0.0.1:2000')

sub = context.socket(zmq.SUB)
sub.connect('tcp://127.0.0.1:2001')
sub.setsockopt(zmq.SUBSCRIBE, b'')

reply = b'a'
isRed = True

import cv2
import numpy as np


def reduce_contrast(frame, alpha=0.5, beta=0):
    corr_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    contrast = 0.80
    brightness = 20
    corr_frame[:,:,2] = np.clip(contrast * corr_frame[:,:,2] + brightness, 0, 255)
    return cv2.cvtColor(corr_frame, cv2.COLOR_HSV2BGR)


def valid_output(prop):
    global current_camera
    if (not bool(prop)):
        return False
    if current_camera == CAMERA_BALL:
        if str(prop["name"])[-4:] != "ball":
            return False
        else:
            return True
    else:
        if str(prop["name"])[:4] != "silo":
            return False
        else:
            return True


def capture_frames(cap1, cap2):
    global frame1, frame2, current_camera
    # cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap1.set(cv2.CAP_PROP_FPS, 30)  # Reduce FPS if needed
    # cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap2.set(cv2.CAP_PROP_FPS, 30)  # Reduce FPS if needed
    while not stop_event.is_set():
        ret1, frame_temp1 = cap1.read()
        ret2, frame_temp2 = cap2.read()
        if not ret1 or not ret2:
            stop_event.set()
            break
        
        with lock:
            frame1 = frame_temp1
            frame2 = frame_temp2

def process_frames():
    global reply, isRed
    global frame1, frame2, current_camera, prev_cam
    while not stop_event.is_set():
        with lock:
            if frame1 is None:
                continue
            if frame2 is None:
                continue
            # frame_copy1 = cv2.resize(frame1.copy(), (256,256))
            # frame_copy2 = cv2.resize(frame2.copy(), (256,256))

        # Perform detection
        prev_cam = current_camera
        frame_copy = None
        if current_camera==CAMERA_BALL:
            frame_copy = frame1
        elif current_camera==CAMERA_SILO:
            frame_copy = reduce_contrast(frame2)

        results = model.predict(source=frame_copy, conf=0.4, imgsz=256, stream=False, show=False)

        # If nothing was detected send empty json object
        # if (len(results[0].boxes.cls) == 0):
        #     prop = {}
        #     json_data = json.dumps(prop)
        #     print(json_data)
        #     print(f"sending {json_data}")
        #     pub.send_json(json_data)

        json_list = []
        # json_list.append({"cam": current_camera})
        # Visualize results on the frame
        for result in results[0].boxes:
            x1_, y1_, w, h = result.xywh[0]
            x1, y1 , x2, y2 = result.xyxy[0]
            confidence = result.conf[0]
            class_id = result.cls[0]
            label = f"{model.names[int(class_id)]} {confidence:.2f}"
            box = [x1_, y1_, w, h]
            box_int = list(map(lambda x: int(x), box))
            prop = {"box": box_int,
                    "conf": result.conf[0].tolist(),
                    "name": str(model.names[int(class_id)])}
            if valid_output(prop):
                json_obj = prop
                if isRed and prop['name'] == 'red_ball':
                    json_list.append(json_obj)
                elif not isRed and prop['name'] == 'blue_ball':
                    json_list.append(json_obj)
                elif isRed == None:
                    if prop['name'] == 'blue_ball':
                        pass
                    elif prop['name'] == 'red_ball': 
                        pass
                    elif prop['name'] == 'purple_ball':
                        pass
                    else:
                        json_list.append(json_obj)

            # Draw bounding box and label
            cv2.rectangle(frame_copy, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame_copy, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # print(f"sending: {json.dumps(json_list)}")
        pub.send_json(json.dumps(json_list))

        # draw ROI
        # cv2.rectangle(frame_copy, (ROI[0]), (ROI[1]), (0,255,0), 2)
        
        # Show the frame
        cv2.imshow('YOLOv8 Object Detection', frame_copy)
        
        # Check for key press to switch cameras or exit
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            stop_event.set()
            break
        # elif key & 0xFF == ord('a'):
        #     current_camera = 0
        # elif key & 0xFF == ord('b'):
        #     current_camera = 2
        # poller = zmq.Poller()
        # poller.register(socket, zmq.POLLIN)

        # socks = dict(poller.poll(5 * 1000))

        # if socket in socks:
        #     try:
        if reply == b'a':
            isRed = True
            current_camera = CAMERA_BALL
        elif reply == b'b':
            isRed = False
            current_camera = CAMERA_BALL
        elif reply == b'c':
            isRed = None
            current_camera = CAMERA_SILO

        #     except IOError:
        #         print("Could not connect to machine")
        # else:
        #     print("Machine did not respond")


def main():
    global reply
    cap1 = cv2.VideoCapture(CAMERA_BALL)  # Camera 1
    cap2 = cv2.VideoCapture(CAMERA_SILO)  # Camera 2

    if not cap1.isOpened() or not cap2.isOpened():
        print("Error: Unable to open one or both video sources.")
        return

    # Start threads
    capture_thread = threading.Thread(target=capture_frames, args=(cap1, cap2))
    process_thread = threading.Thread(target=process_frames)
    
    capture_thread.start()
    process_thread.start()
    
    try:
        while not stop_event.is_set():
            resp = sub.recv()
            print(f"resp {resp}")
            if resp != b'z':
                reply = resp
            # print (reply)
            time.sleep(0.085)
    except KeyboardInterrupt:
        stop_event.set()
    
    # Wait for threads to finish
    capture_thread.join()
    process_thread.join()
    
    # Release video capture and close windows
    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

