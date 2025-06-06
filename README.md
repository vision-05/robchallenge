# Robchallenge
Term 3 Robotics challenge code - Tim Frankel, Thomas Moody, Abdul Azeem Makarim and Yizhong Yan.

The code used to test and control our robot used for the Year 1 Term 3 Robotics Challenge.

# Usage:
To run the line following simulation, download all of the matlab files and add to path, change the parameters of sensor numbers, width, etc in code and then run.

To run the Webots wall following simulation:
- Navigate to the folder `Simulation\Wall Following\worlds`
- Open the world `Wall Following.wbt`
- Build the controller `my_controller.c`
- Run the simulation using the start button at the top of the UI

To run the arduino code, install the qtr library from the ref folder to your `\Documents\Arduino\Libraries` folder or equivalent on UNIX based systems. Then `arduino_code.ino` and upload to the Arduino Giga.

To get real time metrics and operate the robot remotely, use the following packages installed on localhost and connected on the same network:
https://github.com/vision-05/robsense
https://github.com/vision-05/vishon-dev

![image](https://github.com/user-attachments/assets/4f10ea88-5f36-4149-80fb-d24c12ccdc7e)

# Documentation:
The supplied QTR sensors library (for Polulu QTR RC or digital sensor arrays only) has attached documentation either in html or latex form.

# Bugs:
There is a known bug for the Arduino Giga MBedOS that prevents the noInterrupts function being called. This also seems to effect the QTR sensor emitter operation, so the functions which activate the emitters are unusable at present.
