# class for storing the data from sensor_data topic
class Sensors:
    def __init__(self, fsr1, fsr2, fsr3, fsr4, fsr5, fsr6, fsr7, fsr8, fsr9, fsr10, fsr11, fsr12, time):
        self.fsr1 = fsr1
        self.fsr2 = fsr2
        self.fsr3 = fsr3
        self.fsr4 = fsr4
        self.fsr5 = fsr5
        self.fsr6 = fsr6
        self.fsr7 = fsr7
        self.fsr8 = fsr8
        self.fsr9 = fsr9
        self.fsr10 = fsr10
        self.fsr11 = fsr11
        self.fsr12 = fsr12
        self.time = time
