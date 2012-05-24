ledFlicker is a 12-channel LED oscillator for visual experiments. It has
12-bits of color precision per channel and a refresh rate of about 2000 Hz. It
is based on the Arduino Mega microcontroller board. The firmware should also
work with any Arduino Mega clone, like the Seeeduino Mega.

ledFlicker uses pulse width modulation (PWM) to achieve nearly linear intensity
modulation over its entire 12-bit range. It also has a user-configurable gamma
correction table for each channel to allow fine-tuning of intensity linearity.
The gamma calibration is stored on the board's EEPROM and is automatically
loaded when the board boots up. The PWM frequency is about 2KHz and is phase
and frequency correct.

The firmware code is open-source, released under the GPL. You can get the code
from github:

   https://github.com/rfdougherty/ledFlicker

To build the firmware, you need the Arduino environment (we used versions 18
and 19). If you move (or soft-link) the contents of the 'firmware' directory to
your sketchbook directory (~/sketchbook on linux), then the Arduino app will
automatically find it. You also need the Flash library. For convenience, that
is also included in the firmware directory, so you don't need to find and
install it. We also use a modified version of the Messenger library, which is
also included. Just make sure that you have a folder called "libraries" in your
sketchbook directory and that the Flash and Messenger directories are there.
(See the Arduino guide for details.) ledFlicker.pde file should have a parent
folder with the same name. Your sketchbook directory should have at least the
following items:

 sketchbook
   ledFlicker
     ledFlicker.pde
   libraries
     Flash
       Flash.cpp
       Flash.h
     Messenger
       Messenger.cpp
       Messenger.h

To use ledFlicker, simply plug it into a USB port. On modern Linux distros, the
USB driver should already be installed, so the Arduino will appear as a serial
device (e.g., /dev/ttyUSB0) as soon as you plug it in. On Windows and Mac, you
might need to install a USB driver (see the Arduino Guide for more info). If
you have followed the instructions above for installing the full Arduino
environment, then you can skip this step as the driver will have been installed
at that step. 

Once you have the Arduino appearing as a serial device, you control ledFlicker
by sending serial commands at 57600 kbs (baud). Each command must be enclosed
in square brackets ([]) and follows a simple structure, explained below. You
can send multiple commands in one stream of serial data, but don't send too
much at once, since the ledFlicker serial port buffer is somewhat small (128
characters). 

Hardware:
//MODIFY -TY

LedFlicker is built around the [http://arduino.cc/en/Main/ArduinoBoardMega
Arduino Mega] microcontroller board and the
[http://www.luxdrive.com/luxdrive-products/buckpuck-3021-3023-led-driver/
BuckPuck] to provide an adjustable constant-current source for the LEDs. Six of
the Arduino Mega's 16-bit PWM outputs are connected to the external dimming pin
via a PNP transistor. (See Figure 1, which is copied from Figure 12 of the
[http://www.luxdrive.com/download/?dltf&dmid=1109 BuckPuck datasheet].) 
[[Image:Ledflicker_Fig1.png|thumb||right|250px|Figure 1. Schematic for each of
the six channels. The Arduino PWM output is connected to the base of the PNP
transistor via a 5K resistor. The 5K trim pot is used to set the output current
level.]]

Parts List:
* 1 - [http://arduino.cc/en/Main/ArduinoBoardMega Arduino Mega] microcontroller
board
* 6 - high-power LEDs. We used
[http://www.philipslumileds.com/products/luxeon-rebel-color Luxeon Rebels]
pre-mounted to a 20mm star board from [http://www.luxeonstar.com/ Luxeon Star].
The colors we chose (and associated peak wavelength) are Royal Blue (447.5nm),
Blue (470nm), Cyan (505nm), Green (530nm), Amber (590nm), and Red (627nm).
These can be driven at up to 700mA each. The typical forward voltage drop (Vf)
for Red and Amber is 2.9V and for the other colors it is 3.15V.
* 6 - 700mA
[http://www.luxdrive.com/luxdrive-products/buckpuck-3021-3023-led-driver/
BuckPucks] with external dimming. 
* 6 - 2N3906 PNP transistors
* 6 - 25-turn 5 KOhm trim pots (connected between the CTRL and REF BuckPuck
pins for current adjustment)
* A few resistors & capacitors
* Perfboard for mounting things
* Fiber optics and a coupling lens from
[http://www.carclo-optics.com/opticselect/intranet/optics/details/index.php?id_optics=42
Carclo Optics] (e.g.,
[http://www.luxeonstar.com/Carclo-Fiber-Coupling-20mm-Lens-p/10356.htm this
one])
* You'll also need a heatsink to keep the LEDs cool. We used the Thermaltake
TR2-R1, a socket AM2/939/754 CPU heatsink ($15 at Fry's). This one is nice
because it's all aluminum, has a large flat area for mounting the LEDs, has a
relatively quiet fan, and there is a flat bracket on the fan that allows you to
position it upside-down without interfering with the fan. To mount the LEDs, we
drilled 3/32 inch holes and cut threads with a 4-40 plug tap. The LEDs were
then secured with 1/4 inch 4-40 pan-head stainless steel screws, with nylon
washers for electrical insulation.

Suppliers:

* [http://www.sparkfun.com/ SparkFun Electronics]
* [http://ledsupply.com/ LED Supply]
* [http://www.luxeonstar.com/ Luxeon Star]
* [http://thefiberopticstore.com/Specs-Photos.htm The Fiber Optics Store]
* [http://www.digikey.com DigiKey]

Power Requirements:
We use six LEDs, each drawing up to 700mA, for a total current of 6*.7 = 4.2A.
The buckPuck drivers are very efficient (>80%). If we assume 80% efficiency and
know the forward voltage drop for our LEDs (Vf), then we can compute the
maximum power draw with:
 Vf = [2.9, 2.9, 3.15, 3.15, 3.15, 3.15]
 watts = sum((Vf.*0.7))/0.8
Thus, for our set-up, we expect the LEDs to draw a maximum of about 16 Watts.
This obviously exceeds the 2.5 Watt (500mA * 5volt) USB power draw limit, so we
need to power the LEDs with an external power supply. We used a standard 12v
switching supply, like those that power external hard drives. At 12 volts, we
need a supply that can provide at least 1.4 Amps. Note, however, that the
actual Vf for your LEDs might be higher than the "typical" Vf provided in the
spec sheet, so plan accordingly. Also, note that a switching supply might
introduce RF noise in an MR environment. If you plan to place the power supply
within the shielded MR scanner room, you should probably choose a linear power
supply.

To Do:
* Add calibration info to this page. Especially note what we've learned about
LED stability, gamma linearity, and the differences between red/amber LEDs and
the blue/green LEDs.
* Firmware edits:
** keep count of LED hours of service
** store calibration data on-board in EPROM
** auto-sleep LEDs when no commands have been sent in ?? minutes
 
