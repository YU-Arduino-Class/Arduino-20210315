import RPi.GPIO as GPIO
import cv2
import threading
import queue
import time
from time import sleep #time 라이브러리의 sleep함수 사용
import numpy as np
import termios, sys, tty
import pygame
servoPin          = 18   # 서보 핀
SERVO_MAX_DUTY    = 12   # 서보의 최대(180도) 위치의 주기
SERVO_MIN_DUTY    = 3    # 서보의 최소(0도) 위치의 주기
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
# Configurations
GPIO.setup(servoPin, GPIO.OUT)  # 서보핀 출력으로 설정

servo = GPIO.PWM(servoPin, 50)  # 서보핀을 PWM 모드 50Hz로 사용하기 (50Hz > 20ms)
servo.start(0)  # 서보 PWM 시작 duty = 0, duty가 0이면 서보는 동작하지 않는다.



'''
서보 위치 제어 함수
degree에 각도를 입력하면 duty로 변환후 서보 제어(ChangeDutyCycle)
'''
def setServoPos(degree):
  # 각도는 180도를 넘을 수 없다.
  if degree > 180:
    degree = 180

  # 각도(degree)를 duty로 변경한다.
  duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
  # duty 값 출력
  print("Degree: {} to {}(Duty)".format(degree, duty))

  # 변경된 duty값을 서보 pwm에 적용
  servo.ChangeDutyCycle(duty)

def init():
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

def main():
    if getKey('LEFT'):
        print('Key Left was pressed')
    if getKey('RIGHT'):
        print('Key Right was pressed')
    if getKey('UP'):
        print('Key Up was pressed')
    if getKey('DOWN'):
        print('Key Down was pressed')
# 모터 상태
STOP  = 0
FORWARD  = 1
BACKWARD = 2

# 모터 채널
CH1 = 0
CH2 = 1

# PIN 입출력 설정
OUTPUT = 1
INPUT = 0

# PIN 설정
HIGH = 1
LOW = 0

# 실제 핀 정의
#PWM PIN
ENA = 26  #37 pin

#GPIO PIN
IN1 = 19  #37 pin
IN2 = 13  #35 pin

SPEED = 35  # DC 모터 속도



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
    #모터 속도 제어 PWM
    pwm.ChangeDutyCycle(speed)
    if stat == FORWARD:
        GPIO.output(INA, HIGH)
        GPIO.output(INB, LOW)

    #뒤로
    elif stat == BACKWARD:
        GPIO.output(INA, LOW)
        GPIO.output(INB, HIGH)

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


## -------------------------------------------------------------##
## Main Function -----------------------------------------------##
## -------------------------------------------------------------##

# GPIO 모드 설정
GPIO.setmode(GPIO.BCM)

#모터 핀 설정
#핀 설정후 PWM 핸들 얻어옴
pwmA = setMotorPinConfig(ENA, IN1, IN2)

init()

try:
    while True:
        motorspeed = 30
        if getKey('LEFT'):
            servo.ChangeDutyCycle(2.5)
        elif getKey('RIGHT'):
            servo.ChangeDutyCycle(12.5)
        else:
            servo.ChangeDutyCycle(7.5)
        if getKey('UP'):
            setMotor(CH1, motorspeed, FORWARD)
            
        elif getKey('DOWN'):
            setMotor(CH1, motorspeed, BACKWARD)
        else:
            setMotor(CH1, 0, STOP)

        

except KeyboardInterrupt:
        servo.stop()
        GPIO.cleanup()
