
prevError = 0
Kp = 1
Ki = .7
Kd = .07
integral = 0
trageted_qtr_value = 3500

import random
import time

while 1:
    qtr_position = random.randint(0,7000)
    error = trageted_qtr_value - qtr_position

    proportional = Kp * error
    integral += Ki * error
    derivative = Kd * (error - prevError)
    prevError = error

    correction = proportional + integral + derivative

    print(qtr_position, correction)
    time.sleep(1.5)
