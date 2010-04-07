#!/usr/bin/env python

# ledFlicker calibration script
# Copyright 2010 Bob Dougherty (bobd@stanford.edu)

# To install pySerial on Fedora, run "sudo yum install pyserial".
# You may need to change premissions on the serial ports (look in /dev/tty*).

import serial, time, pr650, numpy

ledSer = serial.Serial('/dev/ttyUSB1', 57600, timeout=1)
# default serial params: 8 bits, parity none, stopbits 1
time.sleep(1)
# Display the ledFlicker greeting
out = ledSer.readlines()
for l in out: print(l),
# display help
#ledSer.write('[?]\n');
#out = ledSer.readlines()
#for l in out: print(l),

pr650 = pr650.PR650('/dev/ttyUSB2')

# measure spectra
nRepeats = 10;
spec = numpy.zeros((nRepeats,5,101))
lum = numpy.zeros((nRepeats,5));
for i in range(nRepeats):
    # measure dark spectrum
    ledSer.write('[m,0,0,0,0,0,0]\n');
    time.sleep(0.1)
    pr650.measure()
    lum[i,0] = pr650.getLum()
    [nm,s] = pr650.getSpectrum()
    spec[i,0,:] = s
    # measure red spectrum
    ledSer.write('[m,1,0,0,1,0,0]\n');
    time.sleep(0.1)
    pr650.measure()
    lum[i,1] = pr650.getLum()
    [nm,s] = pr650.getSpectrum()
    spec[i,1,:] = s
    # measure green spectrum
    ledSer.write('[m,0,1,0,0,1,0]\n');
    time.sleep(0.1)
    pr650.measure()
    lum[i,2] = pr650.getLum()
    [nm,s] = pr650.getSpectrum()
    spec[i,2,:] = s
    # measure blue spectrum
    ledSer.write('[m,0,0,1,0,0,1]\n');
    time.sleep(0.1)
    pr650.measure()
    lum[i,3] = pr650.getLum()
    [nm,s] = pr650.getSpectrum()
    spec[i,3,:] = s
    # measure white spectrum
    ledSer.write('[m,1,1,1,1,1,1]\n');
    time.sleep(0.1)
    pr650.measure()
    lum[i,4] = pr650.getLum()
    [nm,s] = pr650.getSpectrum()
    spec[i,4,:] = s

pickle.dump(spec,open('tricolorLed.spec','w'))
pickle.dump(nm,open('tricolorLed.nm','w'))
pickle.dump(lum,open('tricolorLed.lum','w'))

spec = pickle.load(open('tricolorLed.spec','r'))
nm = pickle.load(open('tricolorLed.nm','r'))
lum = pickle.load(open('tricolorLed.lum','r'))

from pylab import *
from numpy import *
specMn = mean(spec,0)
specSd = std(spec,0)
figure
errorbar(nm,specMn[0,:],specSd[0,:],color='k',capsize=0)
errorbar(nm,specMn[1,:],specSd[1,:],color='r',capsize=0)
errorbar(nm,specMn[2,:],specSd[2,:],color='g',capsize=0)
errorbar(nm,specMn[3,:],specSd[3,:],color='b',capsize=0)
errorbar(nm,specMn[4,:],specSd[4,:],color='y',capsize=0)
#'r',nm,specMn[2,:],'g',nm,specMn[3,:],'b',nm,specMn[4,:],'y')
xlabel('Wavelength (nm)')
ylabel('Power (watts/sr/m^2/nm)')
title('Spectral Calibration')
grid(True)
show()

cones = numpy.fromfile('stockman4.txt',sep=' ')
cones = cones.reshape(101,4)
figure
plot(cones[:,0],cones[:,1],'r',cones[:,0],cones[:,2],'g',cones[:,0],cones[:,3],'b')
#'r',nm,specMn[2,:],'g',nm,specMn[3,:],'b',nm,specMn[4,:],'y')
xlabel('Wavelength (nm)')
ylabel('Relative absorption')
title('Cone Fundamentals')
show()

rgb2lms = transpose(cones[:,1:3])*transpose(spectMn[1:3,:])
lms2rgb = linalg.inv(rgb2lms)
rgb2lms = around(rgb2lms/rgb2lms.max()*32767)

stimLMS = array([0,0,1])

lmsBack = dot(rgb2lms,(array([0.5,0.5,0.5])))
# Scale stimulus LMS by the background LMSscaledStimLMS = stimLMS * lmsBack# Determine the stimulus RGB direction stimRGBdir = dot(lms2rgb,scaledStimLMS)stimRGBdir = stimRGBdir/abs(stimRGBdir/stimRGBdir).max()

ledSer = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)
ledSer.write("[w,0,3,0,0.338,-0.2740,0.5209,0,0,0][p]")
ledSer.write("[e,20,.2][w,0,10,0,0.338,-0.2740,0.5209,-0.338,0.2740,-0.5209][p]")

# measure gamma
nRepeats = 10;
for i in range(nRepeats):
    # measure blue spectrum
    ledSer.write('[m,0,0,1,0,0,1]\n');
    time.sleep(0.1)
    pr650.measure()
    lum[i] = pr650.getLum()
    [nm,s] = pr650.getSpectrum()
    spec[i,3,:] = s
f=open('tricolorLed.spec','w')

