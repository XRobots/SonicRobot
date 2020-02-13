# SonicRobot

Code and CAD for my two-wheel balancing robot:  https://www.youtube.com/playlist?list=PLpwJoq86vov_tZ3rsMCH5sylqGT5s9TcU

Main parts in the robot are (more details in part 2 of the video series).

-Teensy 3.6 for the main processor.

-MPU6050 on a GY521 style breakout.

-2x Teensy 3.2 to read the load cells in the legs.

-2x 20Kg Loadcells.

-2x Sparkfun Qwiic Scales: https://www.sparkfun.com/products/15242 .

-'SN65HVD230 CAN BusBoard Network Transceiver Module compatible with PCA82C250' CAN Bus Trancievers.

-2x ODrive 3.6 56V version.

-2x 150Kv 6374 ODrive branded brushless motors.

-2x Turnigy SK3 6374 149Kv brushless motors.

-4x 8192 CPR encoders also availabe from ODrive.

-Some switches etc to initialise the Odrives and an Estop which brings the ODrive RST pins to ground.

-NRF24L01 Rf device.

-Varios proto board.

Main parts in the remote are:

-Arduino MEGA2650.

-Another NRF24L01 RF device.

-some switches, and some 3-axis joysticks I got from eBay.

Other mechanical parts (check part 1 for  more details):

-2x Wheel drive belts - 450mm AT5 belts.

-2x Wheel drive motor pulleys - 14 tooth AT5 aluminium pulleys, 8mm bore to match the motors.

-2x 20mm SFU1605 ball screws

-2x 10mm-10mm CNC couplers

-8x CNC style V-Wheels, 5mm ID bearings, plus nuts and bolts.

-4x M10/10mm Rose joints.

-A bunch of 2040 and 2020 Vslot extrusion - refer to the CAD and video.

-Misc T-nuts and M4 bolts of various lengths.

-8x 12mm ID / 32mm OD / 10mm thick bearings for hip joints.

-8x 8mm ID / 22mm OD / 7mm thick bearings for knee joints.

-Various lengths of M12 and M10 studding plus washers and nuts to suit.

Other details:

-I'm running it off 2x 6S 5000 mAH LiPo packs in series

-Electronics are on a separate 5v regulator running from a separate 11.1v LiPo battery.

-Put 0.1uF decoupling capacitors on everything.

Main Library Dependencies are:

For Teensy CAN: https://github.com/tonton81/IFCT

For the MPU 6050: https://github.com/jrowberg/i2cdevlib

For the load cells: https://github.com/sparkfun/SparkFun_Qwiic_Scale_NAU7802_Arduino_Library/

For the ODrvies: https://github.com/madcowswe/ODrive/tree/master/Arduino/ODriveArduino

Plus RF24 library from the Arduino libraries manager, other built in libraries


