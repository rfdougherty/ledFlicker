#!/usr/bin/env python

# ledFlicker calibration script
# Copyright 2010 Bob Dougherty (bobd@stanford.edu)

# To install necessary modules on Fedora, run "sudo yum install pyserial scipy python-matplotlib".
# You may need to change premissions on the serial ports (look in /dev/tty*).

# standard modules
import os, serial, time, numpy, pylab
# our own pr650 module
import pr650

arduinoDev = '/dev/ttyUSB0'

ledSer = serial.Serial(arduinoDev, 57600, timeout=1)
# Get the arduino serial number
# (ONLY WORKS ON LINUX!)
#out = subprocess.Popen("/sbin/udevadm info -a -n "+arduinoDev+" | grep '{serial}' | head -n1", shell=True, stdout=subprocess.PIPE).communicate()[0]
# or, use the usb module (sudo yum install pyusb): import usb

# default serial params: 8 bits, parity none, stopbits 1
time.sleep(1)
# Display the ledFlicker greeting
out = ledSer.readlines()
for l in out: print(l),
# display help
#ledSer.write('[?]\n');
#out = ledSer.readlines()
#for l in out: print(l),

pr650 = pr650.PR650('/dev/ttyUSB3')

# measure spectra
ledCmd = ['[m,0,0,0,0,0,0]\n',
          '[m,1,0,0,1,0,0]\n',
          '[m,0,1,0,0,1,0]\n',
          '[m,0,0,1,0,0,1]\n',
          '[m,1,1,1,1,1,1]\n']
nRepeats = 20;
specPow = numpy.zeros((101,5,nRepeats))
specLum = numpy.zeros((5,nRepeats));
for i in range(nRepeats):
    for j,cmd in enumerate(ledCmd):
        ledSer.write(cmd);
        time.sleep(0.1)
        pr650.measure()
        specLum[j,i] = pr650.getLum()
        [nm,s] = pr650.getSpectrum()
        specPow[:,j,i] = s

# measure gamma
nRepeats = 4;
nChannels = 3
pwmLevels = numpy.linspace(0,1023,20)/1023
curLevels = numpy.linspace(0,127,10)
gamma = numpy.zeros((nChannels,10,20,nRepeats));
for i in range(nRepeats):
    for j,cur in enumerate(curLevels):
        for k,pwm in enumerate(pwmLevels):
            for chan in range(nChannels):
                tmp = [0,0,0,0,0,0]
                tmp[chan] = cur
                curCmd = '[c,%0.0f,%0.0f,%0.0f]' % (tmp[0],tmp[1],tmp[2])
                tmp[chan] = pwm
                pwmCmd = '[m,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f,%0.6f]' % (tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5])
                ledSer.write(curCmd+pwmCmd+"\n")
                time.sleep(0.1)
                gamma[chan,j,k,i] = pr650.measureLum()



# Save the data in npz format
board = "mega1"
# manufacturer name _ manufacturer product # _ led number (0 or 1?)
ledName = "Kingbright-AAAF5060PBESURVGEC_led0"
calDate = time.strftime("%Y-%m-%d %H:%M:%S")
outName = board+"_led0_"+time.strftime("%y%m%d%H%M")
numpy.savez(outName, board=board, ledName=ledName, calDate=calDate, nm=nm, specLum=specLum, specPow=specPow, pwmLevels=pwmLevels, curLevels=curLevels, gamma=gamma)

# to read the data: 
#  npz = numpy.load(outName)
#  specPow=npz['specPow']

specMn = numpy.mean(specPow,2)
specSd = numpy.std(specPow,2)
col = ['k','r','g','b','y']
pylab.figure
for i in range(5):
    pylab.errorbar(nm,specMn[:,i],specSd[:,i],color=col[i],capsize=0)

pylab.xlabel('Wavelength (nm)')
pylab.ylabel('Power (watts/sr/m^2/nm)')
pylab.title('Spectral Calibration')
pylab.grid(True)
pylab.show()

gammaMn = numpy.mean(gamma,3)
gammaSd = numpy.std(gamma,3)
col = ['r','g','b']
pylab.figure
for i in range(3):
    for j in range(0,10,2):
        pylab.errorbar(pwmLevels,gammaMn[i,j,:],gammaSd[i,j,:],color=col[i],capsize=0)

pylab.xlabel('PWM value')
pylab.ylabel('Luminance (cd/m^2)')
pylab.title('Gamma Calibration')
pylab.grid(True)
pylab.show()

# TODO:
# * Implement a unique board identifier (use USB serial number?)
# * Store mean spectrum, gamma, and rgb2lms in some compact format on the device
# * Compute optimal currents based on calibration data
#   * optimize LMS contrasts?
#   * Set a particular white point?
# * Once the board knows it's rgb2lms, allow intensities to be expressed in LMS 

sensors = numpy.fromfile('stockman4.txt',sep=' ')
sensors = sensors.reshape(101,4)
snm = sensors[:,0]
sensors = sensors[:,1:4]
if numpy.any(snm!=nm)
    raise NameError('Mismatch between sensor wavelengths and measured wavelengths!')
pylab.figure
pylab.plot(nm,sensors[:,0],'r',nm,sensors[:,1],'g',nm,sensors[:,2],'b')
pylab.xlabel('Wavelength (nm)')
pylab.ylabel('Relative absorption')
pylab.title('Cone Fundamentals')
#pylab.show()

rgb2lms = pylab.dot(pylab.transpose(sensors), specMn[:,1:4])
lms2rgb = pylab.linalg.inv(rgb2lms)
rgb2lms = around(rgb2lms/rgb2lms.max()*32767)

stimLMS = numpy.array([0,0,1])
contrast = 1.0

lmsBack = numpy.dot(rgb2lms,(numpy.array([0.5,0.5,0.5])))
# Scale stimulus LMS by the background LMSscaledStimLMS = stimLMS * lmsBack
# Determine the stimulus RGB direction 
# Scale the stimLMS by the lms of the background and multiply by lms2rgbstimRGB = numpy.dot(lms2rgb,stimLMS * lmsBack)
# scale by the max so that it is physically realizablestimRGB = stimRGB/abs(stimRGB).max()*contrast
# compute the actual LMS contrast
actualLMS = numpy.dot(rgb2lms,stimRGB) / lmsBack / 2
print("ActualLMS contrast = [%0.2f, %0.2f, %0.2f]\n" % (actualLMS[0],actualLMS[1],actualLMS[2]) )

# ledSer = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
# Reset means and currents
ledSer.write("[m,0.5,0.5,0.5,0.5,0.5,0.5][c,64,64,64]\n")
cmd = "[e,10,.1][w,0,1,0,%0.4f,%0.4f,%0.4f,0,0,0][p]" % (stimRGB[0],stimRGB[1],stimRGB[2])
ledSer.write(cmd)


