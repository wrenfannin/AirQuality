# Air Quality 
## Prerequisites
 - Latest version of python
 - PySerial library - https://pyserial.readthedocs.io/en/latest/pyserial.html#installation
 - Air quality sensor kit from Dyson - https://www.jamesdysonfoundation.co.uk/resources/secondary-school-resources/engineering-solutions-air-pollution.html
 - Latest version of Python - https://www.python.org/downloads/
 - Free USB port 
## Known Issues
 - On some Linux systems the user does not have permissions to read the serial port - https://support.arduino.cc/hc/en-us/articles/360016495679-Fix-port-access-on-Linux
 - If installing PySerial with pip via a terminal, make sure to [install PySerial and NOT Serial] as they are different libraries
 - If editiing arduino code, there may be an error message caused by having the wrong library version, [ensure to have the V1.0 version of the right library]

## Installation 
 - Assemble the Arduino with the provided shield as in the picture below [IMAGE HERE]
 - Clone this repository :
     - Windows: Download this repository with the releases tab on this github page, make sure to install the .zip file from the latest release. Unpack this .zip file with the compression tool of your choice
     - Linux: Clone this repository with:
         - Arch Based Systems: `sudo pacman -S git` and `git clone https://github.com/wrenfannin/AirQuality/`
         - Ubuntu Based Systems `sudo apt-get git` and `git clone https://github.com/wrenfannin/AirQuality/`
 - Open the extracted folder:
     - Windows: Use file explorer to open the folder in your Downloads
     - Linux: Use `cd ~/AirQuality` or a GUI file manager to navigate to the folder in your home directory
