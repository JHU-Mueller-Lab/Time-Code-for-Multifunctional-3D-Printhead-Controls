# Time-Code-for-Multifunctional-3D-Printhead-Controls

The most up-to-date code is formatted to work with Automation1 (Aerotech) through a TCP/IP network connection.
An example code for using an RS-232 network connection is also provided.

Process:
  \nInput file: GCODE containing auxiliary commands as .txt into T_Code program
  \nOutput file: Uninterrupted GCODE as .txt 
  \nT_Code program will wait for a 'ping' from 3D printer before executing time-synced auxiliary commands

Some notes:
  \nInput G-Code should be in relative (G91)
  \nCurrent aux compatibility: Nordson Ultimus V and WAGO solenoid valve control; can easily adapt to other serial connectivity devices.
