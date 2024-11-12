# Time-Code-for-Multifunctional-3D-Printhead-Controls

The most up-to-date code is formatted to work with Automation1 (Aerotech) through a TCP/IP network connection.

An example code for using an RS-232 network connection is also provided.

## Process:

  - Input file: GCODE containing auxiliary commands as .txt into T_Code program
  
  - Output file: Uninterrupted GCODE as .txt 
  
  - T_Code program will wait for a 'ping' from 3D printer before executing time-synced auxiliary commands

## Some notes:

 - Input G-Code should be in relative (G91)
  
  - Current aux compatibility: Nordson Ultimus V and WAGO solenoid valve control; can easily adapt to other serial connectivity devices.
  
