import cv2
import parinya
from machine import Pin
import time
import numpy as np 

cap = VideoCapture(0)
sensor = ultrasonic.distance_cm()
yolo=parinya.YOLOv3(‘coco.names.txt’,’yolov3-tiny.cfg ’,’yolov3-tiny.weights’)
green light_Pin=Pin(14,Pin_OUT)
yellow light_Pin=Pin(23,Pin_OUT)
red light_Pin=Pin(27,Pin_OUT)
crossgreen light_Pin=Pin(11,Pin_OUT)
crossred light_Pin=Pin(17,Pin_OUT)
p32 = machine.Pin(32, Pin_OUT)
buzzer = machine.PWM(p32)
line_detect=cv2.pointPolygonTest(np.array(area1, np.int32), (int(cx), int(cy)), false)
vid = VideoWriter('cctv.avi', VideoWriter_fourcc(*'MP42'), 25.0, (640,400))

buzzer.freq(98)
height = image.shape[0]
width = image.shape[1]
 
cv2.line(cap, (500, 499), (1449,1128), (255,0,0), 2)
 
While True:
    _, frame = cap.read
    grayframe= cv2.cvtColour(frame, cv2.COLOR_BGR2GRAY)
    Crossred light_Pin.value(1)
    Green light_Pin.value(1)
    if sensor >= 10:
        buzzer.duty(512)
        time.sleep(1)
        buzzer.duty(0)
        green light_Pin.value(0)
        yellow light_Pin.value(1)
        time.sleep(3)
        red light_Pin.value(1)
        time.sleep(5)
        elif class_name ['car','motorbike','bus','truck'] in line_detect <= 0:
            yolo.detect(frame)
            vid.write(frame)
            time.sleep(10)
            elif line_detect > 0:
                red light_Pin.value(1)
                crossred light_Pin.value(1)
                time.sleep(60)
                Buzzer.duty(50)
                Time.sleep(1)
                Buzzer.duty(0)
                time.sleep(2)
                Buzzer.duty(50)
                Time.sleep(1)
                Buzzer.duty(0)
                crossgreen light_Pin.value(1)
                time.sleep(60)
                cv2.imshow(‘frame’, frame)
                if cv2.waitKey(1):
                    break   
    else:
        break
        
vid.release()  

cap.release()
cv2.destroyAllWindows()

-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#Import package 
import RPi.GPIO as GPIO
import time
import numpy
import machine
import signal

#set variable
vid_detect = cv2.VideoCapture(0)
green light_Pin=Pin(14,Pin_OUT)
yellow light_Pin=Pin(23,Pin_OUT)
red light_Pin=Pin(27,Pin_OUT)
crossgreen light_Pin=Pin(11,Pin_OUT)
crossred light_Pin=Pin(17,Pin_OUT)
buzzer = machine.Pin(23, machine.Pin.OUT)
vidfile = VideoWriter('cctv.avi', VideoWriter_fourcc(*'MP42'), 25.0, (640,400))
line_detect=cv2.pointPolygonTest(np.array(area1, np.int32), (int(cx), int(cy)), False)

#set Pin of Trigger(Output) and Echo(Input)
GPIO_TRIGGER =
GPIO_ECHO =

#set gpio input/output pin as BCM
GPIO.setmode(GPIO.BCM)

#set started traffic light
green light_Pin.on()
Crossred light_Pin.on()

#while true loop
while True:
    distance = ultrasonic.distance_cm() #set ultrasonic distance detection as centimetres
    ret, frame = vid_detect.read() #open camera
    if distance <= 10: #set ultrasonic to detect in distance as less than or equal 10 centimetres may show the if loop
        buzzer.duty(512) 
        time.sleep(2)
        Green light_Pin.off()
        yellow light_Pin.on()
        time.sleep(5)
        elif line_detect <= 1: #if it have any object accross the line will be show this loop
            cv2.restangle(frame, (x, y), (x2, y2), colour, 2)
            cv2.rectangle(frame, (x, y), (x + 100, y-30), colour, -1)
            red light_Pin.on()
            vidfile.write(frame)
            elif line_detect > 0: #if it don't have any object accross the line will be show this loop
                red light_Pin.on()
                crossgreen light.on()
                time.sleep(10)
                red light_Pin.off()
                green light_Pin.on()
                crossgreen light.off()
                crossred light.on()                
                cv2.imshow('matthayom watnongkheam school crosswalk cctv01', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    else: #if it not in any option will be break from while true loop
        break

GPIO.cleanup
vidfile.release()

vid_detect.release()
cv2.destroyAllWindows
