
# Import the ADS1x15 module.
no_adc = False      # TODO: remove this "no_adc" stuff when ui is finished
try:
    import Adafruit_ADS1x15 as ADC
except(Exception):
    no_adc = True

import time
import random

from PySide2.QtCore import QObject, Signal

# See ADS1x15 datasheet for more info on gain and addressing
# Gain values corresponding to measurable voltages:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
# As pi pins work on 3.3V it is GAIN = 1
GAIN = 1
# On pi 3 or 4 it is i2c bus 1
BUS = 1

class JoystickSignals(QObject):
    '''
    Defines the signals available from a joystick.

    Supported signals are:

    data
        tuple (id, x, y, z, button_state)

    '''

    data  = Signal(tuple)

class Joystick:
    def __init__(self, id, address = 0x48): # 0x48 is default address for adc1x15
        self.id = id
        self.addr = address
        self.signals = JoystickSignals()
        self.values = [0]*4
        self.center = [828]*4
        self.lower_deadzone= [828]*4
        self.upper_deadzone = [828]*4
        self.min_change = 1

        # Use ADC.ADS1115 for the 16 bit version
        if no_adc == False:
            self.adc = ADC.ADS1015(address=self.addr, busnum=BUS)

    def read(self):
        new_values = [0]*4
        values_changed = False
        if no_adc == False:
            for i in range(4):
                new_values[i] = self.adc.read_adc(i, gain=GAIN)
                # respect to deadzone
                if new_values[i] > self.lower_deadzone[i] and new_values[i] < self.upper_deadzone[i]:
                    new_values[i] = self.center[i]
        else:
            for i in range(4):
                time.sleep(0.1)     # for simulation of read-time to allow other threads to run
                new_values[i] = random.randint(0, 1656)
                # respect to deadzone
                if new_values[i] > self.lower_deadzone[i] and new_values[i] < self.upper_deadzone[i]:
                    new_values[i] = self.center[i]

        #print('I2C: 0x{0:02x}'.format(self.addr), '| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*new_values))
        # only emit if values changed with respect to min_change
        for i in range(0, 4):
            if new_values[i] < self.values[i]-self.min_change or new_values[i] > self.values[i]+self.min_change:
                self.values[i] = new_values[i]
                values_changed = True
        if values_changed == True:
            #print('I2C: 0x{0:02x}'.format(self.id), '| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*self.values))
            self.signals.data.emit((self.id, {'x':self.values[0], 'y':self.values[1], 'z':self.values[2], 'b':self.values[3]}))

    def get_value_set(self):
        return {'x':self.center[0], 'y':self.center[1], 'z':self.center[2], 'b':self.center[3]}

