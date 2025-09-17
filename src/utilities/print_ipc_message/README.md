# Print IPC message

> Technical Responsible: Eduardo P. Abreu

*******
**Tables of contents**

- [Functional Specification](#functional-specification)
- [How to Use](#how-to-use)

*******

> Module Classification: 
<mark style="background-color: blue;color: white;">Utilities</mark> 

## Functional Specification

Prints a message circulating in the system

## How to Use
This module prints a message circulating in the system using its name. When a message is declared, it follows approximately the format below:

```
typedef struct 
{
	int num_motion_commands;
	carmen_robot_and_trailers_motion_command_t *motion_command;
	double timestamp;
	char *host;                 /**< The host from which this message was sent **/
} carmen_robot_ackerman_motion_command_message;
#define      CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_NAME		"carmen_robot_ackerman_motion_command"
#define      CARMEN_ROBOT_ACKERMAN_MOTION_COMMAND_FMT		"{int,<{double,double,double,int, [double:5],double,double,double}:1>,double,string}"
```

First, the message struct is declared, and then there is a define with the message name.
To print this message, execute:
```
./print_ipc_message {msg_name}
```
For example:
```
./print_ipc_message carmen_robot_ackerman_motion_command
```
If it returns erros, you can try the flag "-raw_message":
```
./print_ipc_message carmen_robot_ackerman_motion_command -raw_message
```

### Outputs
```
-----------------------------------------
Message: 'carmen_robot_ackerman_motion_command'; Frequency: 19.211282
int num_motion_commands : 2
carmen_robot_and_trailers_motion_command_t *motion_command :  <{0.000, 0.000, 0.000, 198135808, [0.000, 1758033480.380, 0.000, 0.000, 0.000], 0.000, 0.000, 0.500}, {0.000, 0.000, 0.000, 198135808, [0.000, 1758033480.380, 0.000, 0.000, 0.000], 0.000, 0.000, 0.500}>
double timestamp :  1758033480.380
char *host :  " mpp@lume-Nitro-AN515-55"
```

## Technical Specification

Recovers the format of the message from the IPC and uses its public functions to print the message

