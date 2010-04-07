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
    lum[i] = pr650.getLum()
    [nm,s] = pr650.getSpectrum()
    spec[i,0,:] = s
    # measure red spectrum
    ledSer.write('[m,1,0,0,1,0,0]\n');
    time.sleep(0.1)
    pr650.measure()
    lum[i] = pr650.getLum()
    [nm,s] = pr650.getSpectrum()
    spec[i,1,:] = s
    # measure green spectrum
    ledSer.write('[m,0,1,0,0,1,0]\n');
    time.sleep(0.1)
    pr650.measure()
    lum[i] = pr650.getLum()
    [nm,s] = pr650.getSpectrum()
    spec[i,2,:] = s
    # measure blue spectrum
    ledSer.write('[m,0,0,1,0,0,1]\n');
    time.sleep(0.1)
    pr650.measure()
    lum[i] = pr650.getLum()
    [nm,s] = pr650.getSpectrum()
    spec[i,3,:] = s
    # measure white spectrum
    ledSer.write('[m,1,1,1,1,1,1]\n');
    time.sleep(0.1)
    pr650.measure()
    lum[i] = pr650.getLum()
    [nm,s] = pr650.getSpectrum()
    spec[i,4,:] = s

pickle.dump(spec,open('tricolorLed.spec','w'))
pickle.dump(nm,open('tricolorLed.nm','w'))
pickle.dump(lum,open('tricolorLed.lum','w'))

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

