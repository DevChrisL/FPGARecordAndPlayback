Overview

In this project, you will prototype an audio recorder/player using the ANVYL FPGA Board.   The player must be implemented with picoBlaze soft microcontroller as the main controller.  The design should have the standard features of an audio recorder, namely, ability to record an audio message, play/pause/delete an audio message selected by the user from the audio library.  The user interface is through the Serial Terminal, push buttons, LEDS, and the dip switches.

  
Design Requirements

The following are the design requirements:
R1) On start up, the system must show a welcome message and then display a menu
1)	Play a message
2)	Record a message
3)	Delete a message
4)	Delete all messages
5)	Volume control

R2) For options 1 and 3 above, the system should display the audio library and the user should be able to scroll up or down in the list and then select it to play or delete.

R3) When the memory is full, the system should display a “MEMORY FULL” message.

R4) While playing an audio message, the user be able to pause/resume and skip the message.

R5) While a message is being played or recorded the user should be able to interact with the system through the menu.  In other words, the picoBlaze should not be tied up with playing/recording the message.

R6) The total record time should be at least 4 minutes.

R7) The recorder should be able to record/store atleast 5 messages of variable duration.
 messages (welcome message, memory full message) and commands (play, record, pause, delete, skip, volume control) must be displayed and accepted respectively, via touch screen.  

