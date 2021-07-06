How to use this repository with Git
------------------------------------------------------
ADD THE REPOSITORY 
.- Install Git in your computer 

.- Clone the repository from my GitHub account. The code will be copied in the folder the terminal is located. 
$ git clone https://github.com/aruizgarcia/Firmware

You can now modify the code locally in your computer without affecting any uploaded working code.

--------------------------------------------------------
UPLOAD CHANGES TO THE CODE 
Once the code is modified AND WORKING, you can add the changes to the online repository. 

.- First you can check the status with: 
$ git status

.- Then add the non-staged files with: 
$ git add <files to stage>

.- Finally, make a commit with a descriptive message of the change you implemented
$ git commit -m "Pixhawk-Arduino I2C comms implemented"

---------------------------------------------------------
Branches:

.- stable: Firmware used during the 2020 flight campaign
.- yaw_damper: Firmware used during the 2021 flight campaing which includes a
custom stabilized mode with yaw damper + roll stabilization whose gains can be
tuned from the transmitter. Yaw damper is enabled by setting the parameter
YAW_DAMP_MODE to 1, and the roll stabilization is enabled by setting both
YAW_DAMP_MODE and CUSTOM_STAB_MODE to 1. 
