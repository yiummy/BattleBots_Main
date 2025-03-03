Main Code for Tie-Fighter Battle Bot for Sigma Phi Delta Professional Projects. 

Battle Bot code for Winter 2025 competition. 

Written on VSCode through PlatformIO. Using an ESP32, this code connects a PS4 controller to motor drivers and servos to control the drum weapon and flipper weapon of a 3D printed battle bot. 

PS4 controller inputs are as follows: 
- Left joystick: directional control. Stick will move bot forward and backward, as well as rotate the bot in place. Bot speed will change with joystick inclination.
- Square: toggles the drum motor on and off
- Cross: toggles the flipper up or down
- Circle: toggles safe mode on/off. When off, motors lose power and flipper will remain in its current position.
- PS Button: E-Stop. This will permanently pause motors and flipper will remain in its same position. To resume operation, must reset the ESP32 
