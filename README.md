# Current_Loop_Device
The work methods for this project were split into three main stages: 
 + designing the circuit schematic and the prototype,
 + developing an Arduino software program,
 + manufacturing the physical Printed Circuit Boards (PCB).
The expected inputs such as torque (Nm) and speed (rpm) values were fed into a rotary encoder and were displayed on an LCD I2C screen simul-taneously.
The digital inputs ranged of 0-50Nm for the torque and 0-3000rpm for the speed were converted into the 0-20mA current loops through DACs DFR0552 connected to the Arduino Nano board.
