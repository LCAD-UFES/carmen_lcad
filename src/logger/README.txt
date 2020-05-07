------
USAGE
------

To be writen...


------------------------------------
INCLUDE A NEW MESSAGE IN THE LOGGER
------------------------------------

The logger must subscribe the message, when it receives a message, the handler calls a function that writes the message data on disk.
The message data is storede in the .txt file in one line and the last value must be the time passed since the log started. 
See the example bellow for a camera_message, it has passed 2.162339 seconds since the log started when this message whas received
    CAMERA1_MESSAGE 1 1 1588872682.685808 lcad-G7-7588 921600 640 480 3 1 1  2.162339
    In the case of camera_message, the image is stored in a folder apart with it's path computed from the message timestamp 1588872682.685808


--------------------------------------
INCLUDE A NEW MESSAGE IN THE PLAYBACK
--------------------------------------

The playback reads the lines of the .txt log file and for each one calls a callback based on the first part of the string: CAMERA1_MESSAGE
See the example bellow of the callback for the camera_message.
    {(char *) "CAMERA1_MESSAGE", (char *) CAMERA1_NAME, (converter_func) camera_drivers_read_camera_message_from_log, &camera1_message, 0},
    When the callback finds a line in the .txt log file begining with CAMERA1_MESSAGE, it calls the function that reads the content and publish the message.
    In the callbeck you dont pass the arguments of the read function, only the function name.
        camera_drivers_read_camera_message_from_log(char* string, camera_message* msg)
    The read function must have only two arguments, the string with the current line of the .txt log without the identification, and the message
        In this case, the string content would be: 1 1 1588872682.685808 lcad-G7-7588 921600 640 480 3 1 1  2.162339
    The read function must return a char* with the time passed since the log started: 2.162339
    Use functions CLF_READ_... to read the message content from the string, in your read function.
