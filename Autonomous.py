import RPi.GPIO as GPIO
import cv2 as cv
import numpy as np
import math
import pytesseract
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import smbus

#MESURED PARAMETERS#
focalFactor =1650.83
knownWidth =12 #cm

MIN_PWM = 10  
MAX_PWM = 25 

dt = 0.01 # time step
R = 2.1 #cm
L = 10.5 #cm
V0 =18.7#cm/s


#PID PARAMETERS#
KP = 2.2
KD = 0.1
KI =0.0002


flag= True #let me know if it's the first time I'm entering a turn or if I'm in the midle of turning


class PID(object):
    def __init__(self):
        
        self.kp = KP
        self.ki = KI
        self.kd = KD 
        self.setpoint = 0
        self.error = 0
        self.integral_error = 0
        self.error_last = 0
        self.derivative_error = 0
        self.output = 0
        
    def compute(self, x,y):
        #controlling the angle
        self.error = math.atan2(x,y)
        self.integral_error += self.error * dt
        print("error= "+ str(self.error))
        self.derivative_error = (self.error - self.error_last) / dt
        self.error_last = self.error
        #omega:
        self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error
        print("omega = "+ str(self.output))
        return self.output


class MOTORS(object):
    
    def __init__(self,ain1=12,ain2=13,ena=6,bin1=20,bin2=21,enb=26):
        self.AIN1 = ain1
        self.AIN2 = ain2
        self.BIN1 = bin1
        self.BIN2 = bin2
        self.ENA = ena
        self.ENB = enb
        self.PA  = 15
        self.PB  = 15

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.AIN1,GPIO.OUT)
        GPIO.setup(self.AIN2,GPIO.OUT)
        GPIO.setup(self.BIN1,GPIO.OUT)
        GPIO.setup(self.BIN2,GPIO.OUT)
        GPIO.setup(self.ENA,GPIO.OUT)
        GPIO.setup(self.ENB,GPIO.OUT)
        self.PWMA = GPIO.PWM(self.ENA,500)
        self.PWMB = GPIO.PWM(self.ENB,500)
        self.PWMA.start(self.PA)
        self.PWMB.start(self.PB)
        self.stop()
        
    def fixRight(self,PWM):
        self.PWMA.ChangeDutyCycle(PWM)
        self.PWMB.ChangeDutyCycle(PWM)
        GPIO.output(self.AIN1,GPIO.LOW)
        GPIO.output(self.AIN2,GPIO.HIGH)
        GPIO.output(self.BIN1,GPIO.HIGH)
        GPIO.output(self.BIN2,GPIO.LOW)
        
    def fixLeft(self,PWM):
        self.PWMA.ChangeDutyCycle(PWM)
        self.PWMB.ChangeDutyCycle(PWM)
        GPIO.output(self.AIN1,GPIO.HIGH)
        GPIO.output(self.AIN2,GPIO.LOW)
        GPIO.output(self.BIN1,GPIO.LOW)
        GPIO.output(self.BIN2,GPIO.HIGH)
        
    
    def updatePWM(self,omega,V0):
        rightWheel=(2*V0-omega*L)/(2*R)
        leftWheel=(2*V0+omega*L)/(2*R)
        self.PA = omega2dc(leftWheel)
        self.PB = omega2dc(rightWheel)
        if self.PA<=MIN_PWM:
            self.PA = MIN_PWM
        if self.PB<=MIN_PWM:
            self.PB = MIN_PWM
        if self.PA>= MAX_PWM:
            self.PA= MAX_PWM
        if self.PB>= MAX_PWM:
            self.PB= MAX_PWM

        print('left wheel: '+ str(leftWheel))
        print('PA =' +str(self.PA))
        print('right wheel: '+ str(rightWheel))
        print('PB =' +str(self.PB))
        self.PWMA.ChangeDutyCycle(self.PA)
        self.PWMB.ChangeDutyCycle(self.PB)    
        GPIO.output(self.AIN1,GPIO.LOW)
        GPIO.output(self.AIN2,GPIO.HIGH)
        GPIO.output(self.BIN1,GPIO.LOW)
        GPIO.output(self.BIN2,GPIO.HIGH)

    def stop(self):
        self.PWMA.ChangeDutyCycle(0)
        self.PWMB.ChangeDutyCycle(0)
        GPIO.output(self.AIN1,GPIO.LOW)
        GPIO.output(self.AIN2,GPIO.LOW)
        GPIO.output(self.BIN1,GPIO.LOW)
        GPIO.output(self.BIN2,GPIO.LOW)


def omega2dcPA(omega_L):

    #calculate duty cycle from required frequency of wheel
    if omega_L <=4.76:
        return omega_L/0.476
    elif omega_L<=12.3:
        return (omega_L+2.78)/0.754
    else:
        return (omega_L-1.2)/0.555


def omega2dcPB(omega_R):
    if omega_R <=4.66:
        return omega_R/0.466
    elif omega_R<=11.5:
        return (omega_R+2.18)/0.684
    else:
        return (omega_R+0.4)/0.595

def getDistance(knownWidth, focalLength, perWidth):
    # compute and return the distance from the maker to the camera
    return (knownWidth * focalFactor)/perWidth
    
def getWidth(messuredDistance,focalLength,perWidth):
    return (messuredDistance*perWidth)/focalLength

def stackImages(scale, imgArray):
    
    #Stacks images togrther
    
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),
                                                None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv.cvtColor(imgArray[x][y], cv.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv.cvtColor(imgArray[x], cv.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver

# def empty(a):
#     pass
  
# finding Canny edge detector Parameters #

# cv.namedWindow("Parameters")
# cv.resizeWindow("paramters",640,240)
# cv.createTrackbar("Thresh1","Parameters",0,255,empty)
# cv.createTrackbar("Thresh2","Parameters",255,255,empty)

# finding HSV parameters #

# cv.namedWindow("HSV")
# cv.resizeWindow("HSV",640,240)
# cv.createTrackbar("min HUE","HSV",0,179,empty)
# cv.createTrackbar("max HUE","HSV",179,179,empty)
# cv.createTrackbar("min SAT","HSV",0,255,empty)
# cv.createTrackbar("max SAT","HSV",255,255,empty)
# cv.createTrackbar("min VALUE","HSV",150,255,empty)
# cv.createTrackbar("max VALUE","HSV",255,255,empty)


#HSV parameters
h_min = 25
h_max = 48
s_min = 95
s_max = 255
v_min = 130
v_max = 255

def findSign(contour,contours):
    cx= 0
    d= 0
    width = 0

    for cnt in contours:
        area = cv.contourArea(cnt)
        if area>500:
            #thresholdind the area to avoid uneeded contours
            peri = cv.arcLength(cnt,True)
            approx = cv.approxPolyDP(cnt,0.02*peri,True)
            if len(approx) == 4:
                # then its a sign
                cv.drawContours(contour,cnt,-1,(255,0,255),3)       
                print(area)
                x,y,w,h = cv.boundingRect(approx)
                d = getDistance(knownWidth, focalFactor, w)
                cv.rectangle(contour, (x,y),(x+w,y+h),(0,255,0),5)
                cv.putText(contour, str(d)  ,(contour.shape[1] - 300, contour.shape[0] - 20), cv.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 3)
                cx = int(x+w//2)
                cy = int(y+h//2)
                cv.circle(contour,(cx,cy),5,(0,0,255,cv.FILLED))
                cv.circle(contour,(320,240),5,(255,255,0,cv.FILLED))
                width= getWidth(d,focalFactor,cx-320)
    print("cx = "+str(cx)+" pixels")
    print("cx-320 = "+str(cx-320)+" pixels")
    print("width = "+str(width)+" cm")
    print("d = "+str(d)+" cm")
    return [cx,d,width]

def find_tip(points, convex_hull):
    length = len(points)
    indices = np.setdiff1d(range(length), convex_hull)

    for i in range(2):
        j = indices[i] + 2
        if j > length - 1:
            j = length - j
        if np.all(points[j] == points[indices[i - 1] - 2]):
            return tuple(points[j])
    
def findDirection(image,cx):
    print("activated")
    img_erode = cv.erode(image, np.ones((3,3)), iterations=1)
   
    _,contours,hierarchy = cv.findContours(img_erode, cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
    for cnt in contours:
        peri = cv.arcLength(cnt, True)
        approx = cv.approxPolyDP(cnt, 0.025 * peri, True)
        hull = cv.convexHull(approx, returnPoints=False)
        sides =len(hull)
        if sides == 5 and sides + 2 == len(approx):
            print('number of points: ' + str(len(approx)))
            arrow_tip = find_tip(approx[:,0,:], hull.squeeze())
            if arrow_tip:
                x= arrow_tip[0]
                if x-cx>0:
                    return 'Right'
                else:
                    return 'Left'
        elif len(approx)>=8:
            print(len(approx))
            return 'Stop'
    
    return 'Unrecognized'

            
            
def Procces(Camera,RawCapture,mot,Direction,flag):
    
    pid = PID()
    lastC= 0

    for frame in Camera.capture_continuous(RawCapture, format="bgr", use_video_port=True):
        img = frame.array
        Contour = img.copy()
        imgHSV = cv.cvtColor(img,cv.COLOR_BGR2HSV)

    #     h_min = cv.getTrackbarPos("min HUE","HSV")
    #     h_max = cv.getTrackbarPos("max HUE","HSV")
    #     s_min = cv.getTrackbarPos("min SAT","HSV")
    #     s_max = cv.getTrackbarPos("max SAT","HSV")
    #     v_min = cv.getTrackbarPos("min VALUE","HSV")
    #     v_max = cv.getTrackbarPos("max VALUE","HSV")
        lower = np.array([h_min,s_min,v_min])
        upper = np.array([h_max,s_max,v_max])
        mask = cv.inRange(imgHSV,lower,upper)
        result = cv.bitwise_and(img,img, mask = mask)
        imgBlur = cv.GaussianBlur(result,(5,5),1)
        imgGray = cv.cvtColor(imgBlur,cv.COLOR_BGR2GRAY)
    #     threshold1 =cv.getTrackbarPos("Thresh1","Parameters")
    #     threshold2 =cv.getTrackbarPos("Thresh2","Parameters")
        threshold1 = 0
        threshold2 =250
        imgCanny = cv.Canny(imgGray,threshold1,threshold2)
        kernel = np.ones((5,5))
        imgDil = cv.dilate(imgCanny,kernel, iterations=1)
        _,Contours,hierarchy = cv.findContours(imgDil, cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)
        
        center,distance,width =findSign(Contour,Contours)
        
        
        # if Direction is right or left, it means the robot makes a turn:
        # if Direction is Forward the robot found the sign and now he is tracking 
        
        if Direction == 'Get':
            #read the sign 
            
            Direction = findDirection(imgDil,center)
            print(Direction)
            
        if Direction == 'Stop':
            RawCapture.truncate(0)
            mot.stop()
            return 'Stop'
        
        if Direction == 'Right':
            if flag ==True:
            
                mot.fixRight(10)
                flag= False
                time.sleep(0.6)
                print('taking a turn')
                RawCapture.truncate(0)
                continue                
            
            
            if center<=340 and center !=0:
                print('begin tracking')
                mot.stop()
                Direction = 'Forward'
            
            else:
                mot.fixRight(5)
                print('Still looking for the next sign')
  
        if Direction == 'Left':
            
            if flag ==True:
                mot.fixLeft(10)
                flag= False
                time.sleep(0.6)
                print('taking a turn')
                RawCapture.truncate(0)
                continue
        
            
            if center>=300:
                print('begin tracking')
                mot.stop()
                Direction = 'Forward'
            else:
                print('Still looking for the next sign')
                mot.fixLeft(5)
                
            
                
        if Direction == 'Forward':
            
            if distance == 0:
                #incase the robot lose sight of the sign 
                mot.stop()
                time.sleep(0.1)
                print("no sign in sight!!!")
                if lastC != 0:
                    print(lastC)
                    if lastC>320:
                        #fix left
                        print("fix right")                
                        mot.fixRight(7)
                        time.sleep(0.05)
                    elif lastC<320:
                        #fix right
                        print("fix left")
                        mot.fixLeft(7)
                        time.sleep(0.05)        
            elif distance <= 50:
                #in case of decent proximity to read off the sign
                mot.stop()
                time.sleep(0.2)
                flag=True
                print("distance is "+str(distance))
                RawCapture.truncate(0)    
                return 'Get'
            else:
#                 V0= calcPspeed(distance)
                omega= pid.compute(width,distance)
                mot.updatePWM(omega,V0)
                lastC= center # in case we'll lose sight
            
#         stack = stackImages(0.7,([mask,result],[imgDil,Contour])) 
#         cv.imshow("Stacked Images",stack)
        cv.imshow("image", Contour)
        RawCapture.truncate(0)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        
Mot = MOTORS()   
direction = "Forward"       
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)
while direction != "Stop":
    direction = Procces(camera,rawCapture,Mot,direction,flag)
cv.destroyAllWindows()
camera.close()
GPIO.cleanup()   



