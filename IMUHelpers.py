
RAD_TO_DEG_CONSTANT = 57.29746936176985516473022441508

def printtab(string: str):
    print(string + "\t",end="")

def printtabround(number):
    print(str(round(number,2))+"\t",end="")
def file_write_tab_round(file,number):
    file.write(str(round(number,2))+"\t")
def file_write_tab(file,string):
    file.write(string+"\t")
class IMU_Calibration:
    def __init__(self):
        self.gyro_x_tare = 0.0
        self.gyro_y_tare = 0.0
        self.gyro_z_tare = 0.0
        self.acc_x_tare = 0.0
        self.acc_y_tare = 0.0
        self.acc_z_tare = 0.0


mpu = IMU_Calibration()

