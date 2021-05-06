# User command to set the safety function for the platform

# Safety Function Command Constants
uint8 CMD_NONE = 0
uint8 CMD_AUTOMS_READY = 1
uint8 CMD_AUTOMS = 2
uint8 CMD_MANUAL_READY = 3
uint8 CMD_CRITICAL_STOP1 = 4
uint8 CMD_CRITICAL_STOP2 = 5

std_msgs/Header header

uint8 command    # Set the commands to respective system constants defined above.