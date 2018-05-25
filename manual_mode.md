## The implementations for manual mode  
In manual mode (mode 2) the ES simply passes on lift, roll, pitch, and yaw commands from the joystick and/or keyboard to the drone without taking into account sensor feedback.  
# Joystick part  
We should read the data from joystick for 4 different actions: **lift, pitch, roll and yaw**. Also, the specific value for each action should be given. Furthermore, we would define the **mode switch** in joystick to indicate which button on joystick has the corresponding mode.  
# Keyboard part
We need to have a process for handling the input from keyboard. They are **mode to choose** and **offset to the four actions**.  
# combine the joystick and keyboard  
We have made the function `void create_packet(uint8_t length, uint8_t mode, uint8_t *data, uint8_t *packet)` in `protocol.h` and we will combine the data from **joystick** and **keyboard** to give this input to the packet we made and then transmit the packet to the drone according to the transmit rate we set (maybe 100ms). Before combining the data, we can print the output for the data from joystick and keyboard.  
# Safe mode  
- Using the led on board to indicate the safe mode.  
- Motors should stay still (ae[]=0).  
# Panic mode  
- Using the led on board to indicate the panic mode.  
- The drone should be in a minimum RPM.  
- Decrease the value for the control values (lift, yaw, roll and pitch).  
- After 2 seconds, the drone should be in safe mode.
