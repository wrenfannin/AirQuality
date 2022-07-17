# Air Quality Project - [WIP]
### An Overhaul of JDF's Air Quality Sensor Project
## Prerequisites
 - Latest version of python (3.9+)
 - PySerial library - https://pyserial.readthedocs.io/en/latest/pyserial.html#installation
 - Air quality sensor kit from Dyson - https://www.jamesdysonfoundation.co.uk/resources/secondary-school-resources/engineering-solutions-air-pollution.html
 - Latest version of Python - https://www.python.org/downloads/
 - Free USB port 
 - Arduino IDE - https://www.arduino.cc/en/software
## Known Issues
 - On some Linux systems the user does not have permissions to read the serial port - https://support.arduino.cc/hc/en-us/articles/360016495679-Fix-port-access-on-Linux
 - If installing PySerial with pip via a terminal, make sure to [install PySerial and NOT Serial] as they are different libraries
 - If editiing arduino code, there may be an error message caused by having the wrong library version, [ensure to have the V1.0 version of the right library]

## Installation and Usage
 - Assemble the Arduino with the provided shield as in the picture below [IMAGE HERE]
 - Clone this repository :
     - Windows: Download this repository with the releases tab on this github page, make sure to install the .zip file from the latest release. Unpack this .zip file with the compression tool of your choice
     - Linux: Clone this repository with:
         - Arch Based Systems: `sudo pacman -S git` and `git clone https://github.com/wrenfannin/AirQuality/`
         - Ubuntu Based Systems `sudo apt-get git` and `git clone https://github.com/wrenfannin/AirQuality/`
 - Open the extracted folder:
     - Windows: Use file explorer to open the folder in your Downloads
     - Linux: Use `cd ~/AirQuality` or a GUI file manager to navigate to the folder in your home directory
 - Run the .py file:
     - Windows: Right click the `.py` file in the AirQuality directory to run it
     - Linux: run the .py file in the terminal with `python3 ~/AirQuality<name>.py`
     
 ## Editing Arduino Code
 - Open the .ino file in the Arduino IDE:
     - Windows: Open Arduino.exe and navigate to the file tab to open the .ino file into the IDE, follow prompts on screen to make sure it is able to run
     - Linux: Open the Arduino IDE at `/bin/arduino` and navigate to the file tab to open the .ino file into the IDE, follow prompts on screen to make sure it is able to run
 - Install the correct libraries for the code, they are available in the Arduino IDE's built in library manager
 - Select the correct serial port and the Arduino UNO board in the tools tab, the correct serial number will appear in the dropdown with the name of your Arduino next to it, it will look like either `COM9 (Arduino Uno)` or `/dev/ttyACM0 (Arduino Uno)`
 - Edit the code with either the Arduino IDE or a text editor of your choice
 - Upload the code to your Arduino with the upload button in the top left of the Arduino IDE, if this step fails, check your code or check the 'Known Issues' tab on this page for common errors
 

Copyright (C) 2022 Wren Fannin and Arav Raja <luwr5102@gmail.com> 
Everyone is permitted to copy and distribute copies of this file under GNU-GPL3
