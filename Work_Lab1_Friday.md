# Notes for the Lab1
We were working on the assignment for Lab1 on Friday (04/05/2018). The progress is as following. (**The code modified has been uploaded to the branch Lab1 on GitHub**)

## What we have done
- We have solved the problem that what if the header is missing and another byte is considered as the "header".
  - First bit of each byte is reserved, as only the header sets this bit to 1.

- We have implemented the CRC check successfully. The PC side transmit the data with its CRC check and also the board side can calculate the its CRC check based on the data transmitted.

- We have created the **protocol.c** and **protocol.h**, where we implemented the `create_packet` function.

- When we send a character from PC to board, the pc_terminal can show the header, length, mode, data and CRC correctly, but only for one character.

- Architecture has been redrawn by Eline. :)

## Todo
- Decide the communication rate.

- Implement the protocol from drone to PC.

- Solve the problem of sending characters from PC to drone dynamically. (Not sure about this problem.)

- Make an acknowledgement from drone to PC to tell whether the packet is transmitted correctly. If not, resend it.

- Configuration for the manual mode.
