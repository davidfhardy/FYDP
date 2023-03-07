import serial
import matplotlib.pyplot as plt
import time
import math

###########################################################################################
# Function Defintions
###########################################################################################

cal_coefficient_dB = 15

def ADC_to_volts(y):
    volts = y*3.3/4096
    return volts

def calc_pd_power(y):
    power = y/0.02-92
    return power

def dB_to_dec(y):
    dec = pow(10,y/10)
    return dec

def dec_to_dB(y):
    dB = 10*math.log10(y)
    return dB

def calc_EIRP(y,dist):
    spectrum = []
    for bin in y:
        volts = ADC_to_volts(bin)
        power_pd = calc_pd_power(volts)
        power_dec = dB_to_dec(power_pd)
        spectrum.append(power_dec)
    #get total received power in spectrum
    p_detector = sum(spectrum)
    #turn back into dBm
    p_detector = dec_to_dB(p_detector)
    #multiply by cal coefficient to get power at input of device
    p_input = p_detector-cal_coefficient_dB
    #calculate path loss
    #PL = 20*math.log(3*pow(10,8)/(2.45*pow(10,9)*4*math.pi*dist))
    PL = 0
    EIRP = p_input+PL
    print(EIRP)
    return EIRP

def default(fig):
    ser.open()
    #print(50 * "\n")
    print("\nWaiting for Serial data!")

    ser.reset_input_buffer

    start_saving_values = False
    while start_saving_values == False:
        tmpline = ser.readline()
        tmplist = tmpline.split(b'\r')
        if not tmplist[0]:
            print("\nNo value over Serial!")
            return -1
        elif float(tmplist[0]) == -2.0:
            start_saving_values = True

    # plot setup
    ax = fig.add_subplot(1, 1, 1)
    x = []  # xvals
    y = []  # yvals
    d = 0  # ToF distance
    temp = 0
    r = []  # trend

    print("\nCollecting...")

    time.sleep(0.03)

    last = 0
    base = 0
    for i in range(0, 38):

        yval = 0
        NBA = ser.in_waiting

        #print("\n")
        #print(NBA)

        if NBA < last or NBA > base:
            line = ser.readline()
            newline = line.split(b'\r')
            yval = float(newline[0])

        if i == 0:
            d = yval
            print("distance: {}".format(d))
        elif i == 1:
            temp = yval
            print("temperature: {}".format(temp))
        else:
            xval = 2.375 + 0.00417 * (i - 2)
            x.append(xval)
            y.append(yval)
            r.append(0.5)

        last = NBA

    print("\nGraphing...")

    # a.clear()
    ax.plot(x, y, 'y')
    ax.set_facecolor('k')
    # a.plot( x, r, label="temp if needed")
    # plot setup
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.ylabel('Power (dBm)')
    plt.xlabel('Frequency (GHz)')
    plt.title("PoweRFul Spectrum")
    # plt.legend()

    print("\nDone!")

    fig.canvas.draw()
    fig.canvas.flush_events()

    plt.clf()

    ser.close()
    time.sleep(0.2)
    return 0,y


def main():
    print("\n")
    print("\nSelect a Mode")
    print("\n1. Spectral Graph")
    print("\n2. Peak Spectral Graph")
    mode = int(input("\nEnter Mode: "))

    fig = plt.figure()

    if mode == 1:
        ctn = 0
        while ctn == 0:
            [ctn,y] = default(fig)
            calc_EIRP(y,1)
        if ctn == -1:
            print("\nError!")
            exit()
        else:
            print("\nyou shouldnt be here")
            exit()

    elif mode == 2:
        print("\nMode not implimented yet!")
        exit()

    else:
        print("\nNot a correct input!")
        exit()


###########################################################################################
# Script Beginning
###########################################################################################


# serial initialization
ser = serial.Serial()
ser.port = 'COM3'
ser.baudrate = 115200
ser.bytesize = serial.EIGHTBITS
ser.parity = serial.PARITY_NONE
ser.stopbits = serial.STOPBITS_ONE
ser.timeout = 5

# ser.open()

plt.ion()
if ser.is_open == True:
    print("\n Serial Open with Configuration")
    print(ser, "\n")

if __name__ == "__main__":
    main()
