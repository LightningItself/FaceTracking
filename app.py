import multiprocessing
import cv2
import mediapipe as mp
import time
import numpy as np
from multiprocessing import Process, Queue





# __________pyduino
"""
A library to interface Arduino through serial connection
"""
import serial

class Arduino():

    def __init__(self, serial_port='COM9', baud_rate=9600,
            read_timeout=5):
        self.conn = serial.Serial(serial_port, baud_rate)
        self.conn.timeout = read_timeout

    def set_pin_mode(self, pin_number, mode):
        """
        - I for INPUT
        - O for OUTPUT
        - P for INPUT_PULLUP
        """
        command = (''.join(('M',mode,str(pin_number)))).encode()
        self.conn.write(command)

    def digital_read(self, pin_number):
        command = (''.join(('RD', str(pin_number)))).encode()
        self.conn.write(command)
        line_received = self.conn.readline().decode().strip()
        header, value = line_received.split(':') # e.g. D13:1
        if header == ('D'+ str(pin_number)):
            # If header matches
            return int(value)

    def digital_write(self, pin_number, digital_value):
        command = (''.join(('WD', str(pin_number), ':',
            str(digital_value)))).encode()
        self.conn.write(command) 
     
    def analog_read(self, pin_number):
        command = (''.join(('RA', str(pin_number)))).encode()
        self.conn.write(command) 
        line_received = self.conn.readline().decode().strip()
        header, value = line_received.split(':') # e.g. A4:1
        if header == ('A'+ str(pin_number)):
            # If header matches
            return int(value)

    def analog_write(self, pin_number, analog_value):
        command = (''.join(('WA', str(pin_number), ':',
            str(analog_value)))).encode()
        self.conn.write(command) 

    def close(self):
        self.conn.close()
        print('Connection closed')








flag = multiprocessing.Value('d', 0)
lock = multiprocessing.Lock()



def imageprocess(q) :
    
    wCam, hCam = 640, 480
    cap = cv2.VideoCapture(1)
    cap.set(3, wCam)
    cap.set(4, hCam)
    mpHands = mp.solutions.hands
    hands = mpHands.Hands()
    mpDraw = mp.solutions.drawing_utils

    pTime = 0
    cTime = 0
    while True:
        success, img = cap.read()
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = hands.process(img)
        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                for point in mpHands.HandLandmark:
                    normalizedLandmark = handLms.landmark[point]
                    pixelCoordinatesLandmark = mpDraw._normalized_to_pixel_coordinates(normalizedLandmark.x, normalizedLandmark.y, wCam, hCam)
                    if(pixelCoordinatesLandmark!=None):
                        if(pixelCoordinatesLandmark[0]>wCam*0.7 or pixelCoordinatesLandmark[0]<0.3*wCam or pixelCoordinatesLandmark[1]>hCam*0.7 or pixelCoordinatesLandmark[1]<0.3*hCam):
                            q.put(1)
                            
                        else:
                            q.put(0)
                    else:
                        q.put(0)
                            

                mpDraw.draw_landmarks(img, handLms, mpHands.HAND_CONNECTIONS)

        cTime = time.time()
        fps = 1/(cTime-pTime)
        pTime = cTime

        cv2.putText(img, str(int(fps)), (10,70), cv2.FONT_HERSHEY_PLAIN, 3, (0,0,255), 2)

        cv2.imshow("Image", img)
        cv2.waitKey(1)

def detection(q):
    flag=0
    prev=0
    a = Arduino()
    time.sleep(3)
    PIN = 13
    a.set_pin_mode(PIN, 'O')
    time.sleep(1)
    a.digital_write(PIN, 1)
    time.sleep(1)
    a.digital_write(PIN, 0)
    while(True):
        flag=q.get()
        if flag!=prev:
            if flag:
                print(1)
                a.digital_write(PIN, 1)
            else:
                print(0)
                a.digital_write(PIN, 0)
        prev=flag
        



if __name__ == '__main__':
    q = Queue()
    p1 = Process(target= imageprocess, args=(q,))
    p2 = Process(target=detection, args=(q,))
    p1.start()
    p2.start()
