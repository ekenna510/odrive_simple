# odrive_simple
This is build on odesc 3.6 board should work on original 3.5 or 3.6 idrive board.

Setup your odrive see odrivesetup.py in motorcontrol

wheelbase,wheel diameter and radius hardcoded same with encoder ticks (3200) 
has parameter my_debug used for debuging the code which default to false. With 
has parameter my_debug used for debuging the code which default to false. With  set to true generates simulated encoder ticks

It subscribes to cmd_vel  and set_pose (resets position used by odom) . 

It publishes odom on a timer callback using either the odrive encoder ticks or simulated ones. It also monitors battery voltage and beeps on battery warning and battery low.


If my_debug is false it calls odrive full calibration which will move your robot forward and back.

