# robchallenge
Term 3 Robotics challenge code - Tim Frankel, Thomas Moody, Abdul Azeem Makarim and Yizhong Yan

The majority of code related to our year 1 end of year Robotics Challenge

# usage:
To run the linefollowing simulation, download all of the matlab files and add to path, change the parameters of sensor numbers, width, etc in code and then run

To run the arduino code, install the qtr library from the ref folder to your `\Documents\Arduino\Libraries` folder or equivalent on UNIX based systems. Then `arduino_code.ino` and upload to the Arduino Giga

To get real time metrics and operate the robot remotely, use the following packages installed on localhost and connected on the same network:
https://github.com/vision-05/robsense
https://github.com/vision-05/vishon-dev

![image](https://github.com/user-attachments/assets/4f10ea88-5f36-4149-80fb-d24c12ccdc7e)

# documentation:
The supplied QTR sensors library (for Polulu QTR RC or digital sensor arrays only) has attached documentation either in html or latex form

# bugs:
There is a known bug for the Arduino Giga MBedOS that prevents the noInterrupts function being called. This also seems to effect the QTR sensor emitter operation, so the functions which activate the emitters are unusable at present
