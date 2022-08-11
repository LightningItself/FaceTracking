from gzip import READ
import multiprocessing
import cv2
import mediapipe as mp
import time
import numpy as np
from multiprocessing import Process, Queue
import serial


COM_CHANNEL = 'COM9'
BAUD_RATE = 9600
READ_TIMEOUT = 5

WEBCAM_CHANNEL = 2
MODEL_SELECTION = 0
MODEL_DETECTION_CONFIDENCE = 0.9

TOLERANCE = 0.03

class Arduino():

    def __init__(self, serial_port=COM_CHANNEL, baud_rate=BAUD_RATE,
            read_timeout=READ_TIMEOUT):
        self.conn = serial.Serial(serial_port, baud_rate)
        self.conn.timeout = read_timeout

    def servo_write(self, command):
        self.conn.write(command)
     

class FaceDetection():
    def __init__(self):
        print('Detection Process initiated')
        self.face_detection = mp.solutions.face_detection
        self.drawing = mp.solutions.drawing_utils
        self.drawing_specs = self.drawing.DrawingSpec(thickness=1, circle_radius=1)

    def start(self, show_video, coordinates):
        x_pos=90
        y_pos=90
        
        a = Arduino()
        command = (''.join(('X', str(x_pos)))).encode()
        a.servo_write(command)
        command = (''.join(('Y', str(y_pos)))).encode()
        a.servo_write(command)

        cap = cv2.VideoCapture(WEBCAM_CHANNEL)
        with self.face_detection.FaceDetection(model_selection=MODEL_SELECTION, min_detection_confidence=MODEL_DETECTION_CONFIDENCE) as fd:
            while cap.isOpened():
                success, image = cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    continue
            
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
                image = cv2.flip(image, 1)
                results = fd.process(image)

                
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                
                if results.detections:
                    for detection in results.detections:
                        self.drawing.draw_detection(image, detection)
                        if detection:
                            x=detection.location_data.relative_bounding_box.xmin+detection.location_data.relative_bounding_box.width/2
                            y=detection.location_data.relative_bounding_box.ymin+detection.location_data.relative_bounding_box.height/2
                            coordinates.put([x,y])


                            x_diff = (x-0.4)*5
                            y_diff = (y-0.6)*5
                            print(x_diff, y_diff)
                            if x_diff>9: x_diff=9
                            if x_diff<-9: x_diff=-9
                            if y_diff>9: y_diff=9
                            if y_diff<-9: y_diff=-9
                            
                            x_diff=int(x_diff)
                            y_diff=int(y_diff)
                            if(x_diff):
                                x_pos= int(x_pos+x_diff)
                                if x_pos>170: x_pos=170
                                if x_pos<10: x_pos=10
                                command = (''.join(('X', str(x_pos)))).encode()
                                a.servo_write(command)

                            
                            if(y_diff):
                                y_pos= int(y_pos-y_diff)
                                if y_pos>170: y_pos=170
                                if y_pos<10: y_pos=10
                                command = (''.join(('Y', str(y_pos)))).encode()
                                
                                a.servo_write(command)


        
            
            
        
                            #print([x,y])
                if show_video:
                    cv2.imshow('Face Detection', image)
                if cv2.waitKey(5) & 0xFF ==27:
                    break;
            cap.release()





def hardware_control(q):
    g = 3
    # a = Arduino()
    # x=90
    # y=90

    # while True:
    #     curr_x, curr_y = q.get()
    #     x_diff = (curr_x-0.5)*20
    #     y_diff = (curr_y-0.5)*20
    #     if x_diff>9: x_diff=9
    #     if x_diff<-9: x_diff=-9
    #     if y_diff>9: y_diff=9
    #     if y_diff<-9: y_diff=-9
        
    #     x_diff=int(x_diff)
    #     y_diff=int(y_diff)
    #     if(x_diff):
    #         x= x+x_diff
    #         command = (''.join(('X', str(x)))).encode()
    #         print(command)
        
    #     if(y_diff):
    #         y= y+y_diff
    #         command = (''.join(('Y', str(y)))).encode()
    #         print(command)


        
            
            
        
        



        # if abs(curr_x-prev_x) > TOLERANCE or abs(curr_y-prev_y) > TOLERANCE :
        #     print(str(curr_x)+" " + str(curr_y))
        #     if(curr_x>0.6):
        #         a.servo_write(1, servo_x+2)
        #         servo_x=servo_x+2
        #     elif(curr_x<0.4):
        #         a.servo_write(1, servo_x-2)
        #         servo_x=servo_x-2
        #     if(curr_y>0.6):
        #         a.servo_write(2, servo_y-2)
        #         servo_y=servo_y-2
        #     elif(curr_y<0.4):
        #         a.servo_write(2, servo_y+2)
        #         servo_y=servo_y+2

        #     prev_x = curr_x
        #     prev_y = curr_y



def detection(q) :
    fd = FaceDetection()
    fd.start(show_video=True, coordinates=q)


if __name__ == '__main__':
    q = Queue()
    detection_process = Process(target=detection, args=(q, ))
    hardware_process = Process(target=hardware_control, args=(q, ))
    detection_process.start()
    hardware_process.start()
