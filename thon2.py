#Import package 
import RPi.GPIO as GPIO
import time
import numpy
import machine

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
        elif line_detect <= 10: #if ultrasonic detect object less than or equal 10 centimetres may show the elif loop
            cv2.restangle(frame, (x, y), (x2, y2), colour, 2)
            cv2.rectangle(frame, (x, y), (x + 100, y-30), colour, -1)
            red light_Pin.on()
            vidfile.write(frame)
            elif line_detect > 10: #if ultrasonic detect object more than 10 centimetres may show the elif result
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

vidfile.release()

vid_detect.release()
cv2.destroyAllWindows
