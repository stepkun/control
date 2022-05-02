# Test programm for two joystick modules
# author: Stephan Kunz
# based on Tony DiCola's sample program
#
# install libraries:
# sudo pip install adafruit-ads1x15
#

import time

# Import the ADS1x15 module.
import Adafruit_ADS1x15 as ADC

# See ADS1015/ADS1115 datasheet for more info on gain and addressing
# Gain values corresponding to measurable voltages:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
# As pi pins work on 3.3V it is GAIN = 1
GAIN = 1
JOY_1 = 0x48
JOY_2 = 0x49
# on pi 3 or 4 it is i2c bus 1
BUS = 1

def main(args=None):
    # Create an ADS1015 ADC instance for each joystick
    # Use ADC.ADS1115 for the 16 bit version
    adc1 = ADC.ADS1015(address=JOY_1, busnum=BUS)
    adc2 = ADC.ADS1015(address=JOY_2, busnum=BUS)

    print('Reading ADS1x15 values, press Ctrl-C to quit...')
    # Print nice channel column headers.
    print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |  {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*range(8)))
    print('-' * 73)
    # Read all the ADC channel values of both ADC's in a list.
    values1 = [0]*4
    values2 = [0]*4
    # Main loop.
    while True:
        # Read the specified ADC channel using the previously set gain value.
        # Note you can also pass in an optional data_rate parameter that controls
        # the ADC conversion time (in samples/second). Each chip has a different
        # set of allowed data rate values, see datasheet Table 9 config register
        # DR bit values.
        #values[i] = adc.read_adc(i, gain=GAIN, data_rate=128)
        # Each value will be a 12 or 16 bit signed integer value depending on the
        # ADC (ADS1015 = 12-bit, ADS1115 = 16-bit).
        for i in range(4):
            values1[i] = adc1.read_adc(i, gain=GAIN)
            values2[i] = adc2.read_adc(i, gain=GAIN)

        # Print the ADC values.
        print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values1), ' {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values2))
        # Pause a short time, not less than 0.2 seconds
        time.sleep(0.2)

if __name__ == '__main__':
    main()
