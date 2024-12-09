from machine import Pin, SoftI2C
import time
import ustruct


class SensorBar:
    lastBarValue = 0
    lastBarRawValue = b'\x00'
    def __init__(self,address = 0x3E, resetPin = 255, interruptPin = 255, oscillatorPin = 255,sda = 10,scl = 11):
        self.deviceAddress = address
        self.pinInterrupt = interruptPin
        self.pinOscillator = oscillatorPin
        self.pinReset = resetPin
        self.invertBits = 0
        self.barStrobe = 0
        sdaPin = Pin(sda)
        sclPin = Pin(scl)
        self.i2c = SoftI2C(sclPin,sdaPin)

    def begin(self):
        returnVar = 0

        if (self.pinInterrupt != 255):
            self.pinInterrupt = Pin(255,Pin.IN,Pin.PULL_UP)
        
        self.i2c.start()
        self.reset()

        testRegisters = 0
        testRegisters = self.i2c.readfrom_mem(self.deviceAddress,0x13,2)
        
        if testRegisters == b'\xFF\x00':
            #print("works")
            self.i2c.writeto_mem(self.deviceAddress,0x0F,b'\xFF')
            self.i2c.writeto_mem(self.deviceAddress,0x0E,b'\xFC')
            self.i2c.writeto_mem(self.deviceAddress,0x10,b'\x01')
            returnVar = 1
        else:
            returnVar = 0
        
        return returnVar

    def reset(self):
        #bytes(0x12)
        self.i2c.writeto_mem(self.deviceAddress,0x7D,b'\x12')
        self.i2c.writeto_mem(self.deviceAddress,0x7D,b'\x34')

    def SetInvertBits(self):
        self.invertBits = 1
    def ClearInvertBits(self):
        self.invertBits = 0
    def SetBarStrobe(self):
        self.barStrobe = 1
    def ClearBarStrobe(self):
        self.barStrobe = 0

    def scan(self):
        if self.barStrobe== 1:
            self.i2c.writeto_mem(self.deviceAddress,0x10,b'\x02')
            time.sleep_ms(2)
            self.i2c.writeto_mem(self.deviceAddress,0x10,b'\x00')
        else:
            self.i2c.writeto_mem(self.deviceAddress,0x10,b'\x00')
        
        self.lastBarRawValue = self.i2c.readfrom_mem(self.deviceAddress,0x11,1)

        if self.invertBits == 1:
            barVal = int.from_bytes(self.lastBarRawValue,"big")
            invertVal = int.from_bytes(b'\xFF',"big")
            result = barVal ^ invertVal 

            self.lastBarRawValue = result.to_bytes(1,'big')

            

        if self.barStrobe == 1:
            self.i2c.writeto_mem(self.deviceAddress,0x10,b'\x03')
        
    
    def GetRaw(self):
        self.scan()
        return self.lastBarRawValue

    def GetPosition(self):
        accumulator = 0
        bitsCounted = 0
        i = 0
        self.scan()

        for i in range(8):
            if ((int.from_bytes(self.lastBarRawValue,"big") >> i) & int.from_bytes(b'\x01',"big") == 1): # type: ignore
                bitsCounted = bitsCounted + 1
                
        i = 7
        for i in range(7,3,-1):
            if ((int.from_bytes(self.lastBarRawValue,"big") >> i) & int.from_bytes(b'\x01',"big") == 1):
                accumulator = accumulator + ((-32 * (i-3)) + 1)
        i = 0
        for i in range(4):
            if ((int.from_bytes(self.lastBarRawValue,"big") >> i) & int.from_bytes(b'\x01',"big") == 1):
                accumulator = accumulator + ((32 * (4-i)) - 1)
        
        if(bitsCounted > 0):
            self.lastBarValue = accumulator/bitsCounted
        else:
            self.lastBarValue = 0
        
        return self.lastBarValue

