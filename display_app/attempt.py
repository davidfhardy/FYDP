
import matplotlib.pyplot as plot
import matplotlib.animation as animation
from matplotlib import style
import numpy as num
import random
import serial

#serial initialization
ser = serial.Serial()
ser.port = 'COM3'
ser.baudrate = 115200
ser.bytesize = serial.SEVENBITS
ser.parity = serial.PARITY_EVEN
ser.stopbits = serial.STOPBITS_ONE
ser.timeout = 10

ser.open()

if ser.is_open == True: 
    print("\n Serial Open with Configuration")
    print(ser, "\n")

#setup plot with matlab lib
spectrum = plot.figure()
a = spectrum.add_subplot(1,1,1)
x = [] #frequency bins
y = [] #spectral power / voltage currently
r = [] #trend if needed

countset = 0

#animation 
def animate(i, x, y, cs):
    
    #get data from serial
    print("data from com")
    print(ser.readline())
    line = ser.readline()
    linelist = line.split(b'\r')
    #if isinstance(linelist[0], float) == False:
    #    while True:
    #        number = int(input("Enter 0 to exit: "))
    #        if number == 0:
    #            break
    #    print("No more values")
    #    exit()
    realvalue = float(linelist[0])
    xval = 4.0 + (i*0.5)
    i = cs
    i = i + 1
    cs = i

    #add x & y to list
    x.append(xval)
    y.append(realvalue)
    r.append(0.5)

    #limits if needed
    #x = x[-20:]
    #y = y[-20:]

    #draw
    a.clear()
    a.plot( x, y, label="Voltage vs Freq")
    a.plot( x, r, label="temp if needed")

    #plot setup
    plot.xticks(rotation=45, ha='right')
    plot.subplots_adjust(bottom=0.30)
    plot.title('tba')
    plot.ylabel('voltage')
    plot.xlabel('frequency')
    plot.legend()
    plot.axis([1,5,0,1.1]) #trails indicated by second argument

ani = animation.FuncAnimation(spectrum, animate, fargs=(x, y, countset), interval=250)
plot.show()

print("done")
