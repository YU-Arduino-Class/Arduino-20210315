import RPi.GPIO as GPIO
import cv2
import threading
import time
import numpy as np
import pygame
import os

# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

'''
DC, 서보 모터 설정
'''
# 모터 상태
STOP  = 0
FORWARD  = 1
BACKWARD = 2

# 모터 채널
CH1 = 0
CH2 = 1

# 실제 핀 정의
#PWM PIN
ENA = 26 #TODO #37 pin

#GPIO PIN
IN1 = 19 #TODO #37 pin
IN2 = 13 #TODO #35 pin

### Servo
SERVO_PIN = 18 #TODO # 서보 핀

# 핀 설정 함수
def setMotorPinConfig(EN, INA, INB):
    GPIO.setup(EN, GPIO.OUT)
    GPIO.setup(INA, GPIO.OUT)
    GPIO.setup(INB, GPIO.OUT)
    # 100hz 로 PWM 동작 시킴
    pwm = GPIO.PWM(EN, 100)
    # 우선 PWM 멈춤.
    pwm.start(0)
    return pwm

# 모터 제어 함수
def setMotorContorl(pwm, INA, INB, speed, stat):
    HIGH = 1
    LOW = 0
    #모터 속도 제어 PWM
    pwm.ChangeDutyCycle(speed)
    if stat == FORWARD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)

    #뒤로
    elif stat == BACKWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)

    #정지
    elif stat == STOP:
        GPIO.output(INA, LOW)
        GPIO.output(INB, LOW)


# 모터 제어함수 간단하게 사용하기 위해 한번더 래핑(감쌈)
def setMotor(ch, speed, stat):
    if ch == CH1:
        #pwmA는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorContorl(pwmA, IN1, IN2, speed, stat)
    else:
        #pwmB는 핀 설정 후 pwm 핸들을 리턴 받은 값이다.
        setMotorContorl(pwmB, IN3, IN4, speed, stat)

def setServoMotor(duty):
    servo.ChangeDutyCycle(duty)


GPIO.setup(SERVO_PIN, GPIO.OUT)
servo = GPIO.PWM(SERVO_PIN, 50) # 서보핀을 PWM 모드 50Hz로 사용하기 (50Hz > 20ms)
servo.start(0)

#모터 핀 설정
pwmA = setMotorPinConfig(ENA, IN1, IN2)

'''
키보드 설정
'''

def keyboard_init():
    pygame.init()
    win = pygame.display.set_mode((100, 100))

def getKey(keyName):
    ans = False
    for eve in pygame.event.get():pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame,'K_{}'.format(keyName))
    if keyInput [myKey]:
        ans = True
    pygame.display.update()

    return ans

keyboard_init()

'''
초음파 설정
'''

TRIG = 7 #TODO
ECHO = 8 #TODO

GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

GPIO.output(TRIG, False)
print("Waiting for sensor to settle")
time.sleep(2)

distance = 1000

def sonar_thread():
    global distance
    while True:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        start = 0
        while GPIO.input(ECHO)==0:
            start = time.time()	 # Echo 핀 상승 시간
        while GPIO.input(ECHO)==1:
            stop= time.time()	 # Echo 핀 하강 시간

        check_time = stop - start
        distance = check_time*34300/2 #TODO: 거리 계산하는는 코드 넣기
        print("Distance : %.1f cm" % distance)

        time.sleep(0.2)


'''
카메라 설정
'''

# 학습한 정보 분류기에 넣어주기
recognizer = cv2.face.LBPHFaceRecognizer_create()
recognizer.read('preparation_camera/trainer/trainer.yml')
cascadePath = "preparation_camera/haarcascade_frontalface_default.xml"
faceCascade = cv2.CascadeClassifier(cascadePath);

font = cv2.FONT_HERSHEY_SIMPLEX

#iniciate id counter
ids = 0

# ID 별 이름 0, 1, 2, 3 순서
#         ['None', '<1번이름>', '<2번이름>']
# 첫번째는 'None'으로 놔둔다 (우리 iD는 1번에서 시작함)
names = ['None', 'Younggun', 'MG','minchang'] #TODO: 여러분들이 저장한 사람들 이름

# 카메라 시작하기
cam = cv2.VideoCapture(0)
cam.set(3, 640) # set video widht
cam.set(4, 480) # set video height

# 얼굴은 최소한 원래 이미지 크기의 10%는 되어야 함
minW = 0.1*cam.get(3)
minH = 0.1*cam.get(4)
count = 0

def camera_thread():
    while True:
        global count
        ret, img =cam.read()

        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(
            gray,
            scaleFactor = 1.2,
            minNeighbors = 5,
            minSize = (int(minW), int(minH)),
           )

        for(x,y,w,h) in faces:
            cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)

            ids, difference = recognizer.predict(gray[y:y+h,x:x+w])
            probability = round(100 - difference)
            # difference로 검출 되었는지 확인
            if (difference < 100):
                id_name = names[ids]
                confidence = "  {0}%".format(round(100 - difference))
            else:
                id_name = "unknown"
                confidence = "  {0}%".format(round(100 - difference))

            cv2.putText(img, str(id_name), (x+5,y-5), font, 1, (255,255,255), 2)
            cv2.putText(img, str(confidence), (x+5,y+h-5), font, 1, (255,255,0), 1)

            if count < 10 and ids>0 and probability>10: # TODO, ids가 0 초과, probability가 10 초과인 조건 추가
                cv2.imwrite('result/{}_{}_{}_{}.png'.format(count, ids, id_name, confidence), img)
                count = count + 1

        cv2.imshow('camera',img)

        k = cv2.waitKey(10) & 0xff # Press 'ESC' for exiting video
        if k == 27:
            break


## -------------------------------------------------------------##
## Main Function -----------------------------------------------##
## 초음파와 카메라는 아래 thread라는 기능으로 계속 동작하게 됩니다-##
## -------------------------------------------------------------##

## Thread for sonar (초음파 쓰레드)
thread1= threading.Thread(target=sonar_thread)
thread1.daemon=True  	# 데몬 쓰레드란 백그라운드에서 실행되는
                    # 쓰레드로 메인 쓰레드가 종료되면 즉시 종료되는 쓰레드이다.
thread1.start()

## Thread for sonar (카메라 쓰레드)
thread2 = threading.Thread(target=camera_thread)
thread2.daemon=True  	# 데몬 쓰레드란 백그라운드에서 실행되는
                    # 쓰레드로 메인 쓰레드가 종료되면 즉시 종료되는 쓰레드이다.
thread2.start()


'''
구간1:
 - 정지상태에서 대기하고 있다가 스페이스 누르면 직진 시작
 - distance (초음파 거리)가 50cm이하면 정지
 - dc_speed1을 잘 설정해서 벽에 박지 않도록 주의 (너무 빠르면 초음파로 감지해도 충돌)
'''

dc_speed1 = 30 #TODO: 구간 1에서 직진시 속도

#TODO: 서보모터 직진방향만들기
#setServoMotor()

# 스페이스바 눌릴때 까지 기다리기
while True:
    if getKey('SPACE'):
        print('Start!')
        break

    time.sleep(0.1)


# 직진 시작! -> 특정 조건이 되면 멈추기
while True:
    setMotor(CH1, dc_speed1, FORWARD)
    if distance<=50: #TODO: 조건문을 distnace가 50이하면 정지하는 조건 추가
        setMotor(CH1, 0, STOP)
        break

    time.sleep(0.1)


'''
구간2:
 - 키보드로 조종하는 부분
'''

dc_speed2 = 30 #TODO: 키보드로 조정시 속도
left_duty = 4 #TODO: 왼쪽 키보드 눌렀을 때 서보 듀티
centor_duty = 5 #TODO: 중앙 서보 듀티
right_duty = 7 #TODO: 오른쪽 키보드 눌렀을 때 서보 듀티

while True: #TODO: 전, 후, 좌, 우 키보드 눌렀을 때 동작하는 코드 넣기
    if getKey('LEFT'):
        print('Key Left was pressed')
        servo.ChangeDutyCycle(4.5)
        #TODO
    elif getKey('RIGHT'):
        print('Key Right was pressed')
        servo.ChangeDutyCycle(9.5)
        #TODO
    elif getKey('UP'):
        print('Key Up was pressed')
        setMotor(CH1, dc_speed2, FORWARD)
        #TODO
    elif getKey('DOWN'):
        print('Key Down was pressed')
        setMotor(CH1, dc_speed2, BACKWARD)
        #TODO
    elif getKey('ESCAPE'):
        print('Key ESC was pressed')
        print('QUIT Program')
        #TODO: 'ESC'누르면 종료하는 부분
        servo.ChangeDutyCycle(7.5)
        setMotor(CH1, 0, STOP)#TODO: 서보모터 직진방향 정렬, DC모터 정지
        time.sleep(1)
        break
    else:
        servo.ChangeDutyCycle(7.5)#TODO: 서보모터 직진방향 정렬
        setMotor(CH1, 0, STOP)

    time.sleep(0.1)

'''
구간3:
 - 구간 2의 연장 (구간 3은 따로 코드 작성 필요 없음, 카메라 코드 추가 시 자동으로 작동)
 - camera_thread는 처음부터 자동 실행중이므로 RC카 조종해서 사진이 잘 나오도록 자동차 위치 시키기
 - 다른 팀원이 원격으로 사진 보고 있다가 알려주기
'''

servo.stop()
GPIO.cleanup()
thread1.join()
thread2.join()
