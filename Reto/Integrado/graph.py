import serial.tools.list_ports
import struct
import cmath
import matplotlib.pyplot as plt
import scipy.fftpack
import numpy as np
import time

ports = serial.tools.list_ports.comports()

portList = []
for onePort in ports:
    portList.append(str(onePort))
    print(onePort)

s = serial.Serial(
    port = 'COM8', 
    baudrate = 115200, 
    timeout = 0, 
    xonxoff = True)
print(s)

s.flushInput()
s.flushOutput()
s.close()
s.open()

val = []
fft = []

N = 128                     # Número de muestras
Fs = 128                    # Frecuencia de mestreo [Hz]
T = 1/Fs;                   # Periodo de muestreo [s]      
L = 128;                      # Longitud de la señal [s]
t = np.linspace(0, (N-1)*T, N)       # Time vector
f = np.linspace(0, (N-1)*(Fs/N), N)
f_plot = f[0:int(N/2+1)]

flag = 1;

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)

while(True):
    if(s.in_waiting and s.is_open):
        num = s.readline()
        print(num)
        """
        vF, = struct.unpack('<f', num)

        val.append(vF)
        print(len(val))

        if(len(val) == N*2):
            for i in range(0, N*2, 2):
                fft.append(complex(val[i], val[i + 1]))
            val = []
            print(fft)

            
            if(flag == 1):
                x_mag = np.abs(np.array(fft)) / N
                x_mag_plot = 2 * x_mag[0:int(N/2+1)]
                x_mag_plot[0] = x_mag_plot[0] / 2
                line1, = ax.plot(f_plot, x_mag_plot)
                time.sleep(2.5)
                flag == 0
            else:
                x_mag = np.abs(np.array(fft)) / N
                x_mag_plot = 2 * x_mag[0:int(N/2+1)]
                x_mag_plot[0] = x_mag_plot[0] / 2
                line1.set_ydata(x_mag_plot)
                fig.canvas.draw()
                fig.canvas.flush_events()
                time.sleep(2.5)"""