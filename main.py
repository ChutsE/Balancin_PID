from machine import Pin, PWM, I2C
from vl53l0x import VL53L0X
import time
import math

set_point = 0
kp = 25
ki = 10
kd = 10
MAX = 873

sample_time_ms = 30
timeprev = 0
PID = 0
pid_d = 0
pid_i = 0
pid_p = 0
R = 261
prev_error = 0

# Set up VL53L0X broadcast
I2C_bus = I2C(0, sda=Pin(21), scl=Pin(22))
tof = VL53L0X(I2C_bus)

# Set up PWM
elice_up = PWM(Pin(19))
elice_down = PWM(Pin(18))

while True:
    dt = (time.time_ns() - timeprev) / 1000000000
    timeprev = time.time_ns()
    if dt > 2*sample_time_ms/1000:
        dt = 2*sample_time_ms/1000

    tof.start()
    distance = tof.read() - 30
    tof.stop()

    theta = (2 * math.atan(distance / R) - math.pi / 2) * (180 / math.pi)

    error = theta - set_point

    pid_p = kp * error
    pid_i = pid_i + ki * error * dt
    pid_d = kd * ((error - prev_error) / dt)
    prev_error = theta - set_point

    PID = int(pid_p + pid_i + pid_d)

    if PID > MAX:
        PID = MAX
    if PID < -MAX:
        PID = -MAX

    if PID < 0:
        elice_down.duty(0)
        elice_up.duty(-PID + 150)
    elif PID > 0:
        elice_up.duty(0)
        elice_down.duty(PID + 150)

    print((theta, set_point))
    time.sleep_ms(sample_time_ms)



