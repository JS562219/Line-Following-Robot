from machine import Pin, PWM, ADC, SoftI2C
import time
import ustruct
import math
import uasyncio as asyncio

from microPID import PID
from sensorbar import SensorBar
from hcsr04 import HCSR04

from hardware_setup import ssd  # Create a display instance
from gui.core.ugui import Screen
from gui.core.writer import CWriter
from gui.core.colors import *

from gui.widgets import Label, CloseButton
import gui.fonts.freesans20 as font

from IMUHelpers import printtab, printtabround
global file


# Motor Pins
ENA = PWM(Pin(6),freq=1000)
IN1 = Pin(7, Pin.OUT)
IN2 = Pin(8, Pin.OUT)

IN3 = Pin(10, Pin.OUT)
IN4 = Pin(11, Pin.OUT)
ENB = PWM(Pin(12),freq=1000)

# Motor H-Bridge Control
IN2.on()
IN1.off()

IN4.on()
IN3.off()

# SensorBar Pins
mySensor = SensorBar(sda=2,scl=3)
mySensor.begin()

# Ultrasonic Pins
trigger_pin = Pin(14, Pin.OUT)
echo_pin = Pin(15, Pin.OUT)
Usensor = HCSR04(trigger_pin, echo_pin)

# PID Programming
dt = .01
# Motor PID
controller = PID(kp = 250, ki = 1, kd = 2500, setpoint = 0, output_limits=(-10000,10000))

# Ultrasonic Sensor PID
ultra_controller = PID(kp = 650, ki = 1, kd = 1, setpoint = 20, output_limits=(0,35000))

# Data Collection
file = open("DataPoint.csv",'w')

printtab("Motor Control")
printtab("Ultra Sonic Control")
file.write("Motor Control"+"\t")
file.write("Ultra Sonic Control"+"\t")

async def client(dist,motorL,motorR):

    while 1:

        # Base Motor Speed
        setSpeed = 20000

        # Motor PID
        inputVar = mySensor.GetPosition()
        output = controller.compute(inputVar,dt)
        print("Motor PID Output:", output)

        # Ultrasonic Distance Reading
        USdist = Usensor.distance_cm()


        Uoutput = ultra_controller.compute(USdist,dt)
        #print("Distance: ", USdist)
        #print(" ")
        print("US PID Output:", Uoutput)
        
        # Motor Control
        ENA.duty_u16(max((int((setSpeed + output) - Uoutput)),0))
        ENB.duty_u16(max((int((setSpeed - output) - Uoutput)),0))




        # Prints Left and Right Wheel Speed
        print("Motor Speed L,R", int((setSpeed + output) - Uoutput), int((setSpeed - output) - Uoutput))


        # Pico Explorer Display
        dist.value(str(USdist))
        motorL.value(str(round((setSpeed + output) - Uoutput)))
        motorR.value(str(round((setSpeed - output) - Uoutput)))
        # time.sleep(dt)

        printtabround(output)
        printtabround(Uoutput)        
        file.write(str(round(output,2))+"\t")
        file.write(str(round(Uoutput,2))+"\t")

        file.write("\n")

        await asyncio.sleep(dt)

 
class BaseScreen(Screen):
    def __init__(self):
        super().__init__()
        labels = {'bdcolor' : RED,
                'fgcolor' : WHITE,
                'bgcolor' : DARKGREEN,
                'justify' : Label.CENTRE,
                }
        
        wri = CWriter(ssd, font, GREEN, BLACK)  # verbose = True

        lbl = Label(wri,4,4,text='            ')
        lbl_txt1 = Label(wri,2, 80, 'dist (cm)')

        lbl_ML = Label(wri,29,4,text='            ')
        lbl_txt2 = Label(wri,29,80,text='LMS')

        lbl_MR = Label(wri,54,4,text='            ')
        lbl_txt3 = Label(wri,54,80,text='RMS')

        self.reg_task(client(lbl,lbl_ML,lbl_MR))

        CloseButton(wri)
    
Screen.change(BaseScreen)



