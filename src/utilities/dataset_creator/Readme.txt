
How to use:
1. Use the convert_log_images app to extract the images from the log as ".png" images and store them in a directory.
2. Use the following command to create an initial dataset file called input.txt:
OBS: REPLACE <dir> WITH THE NAME OF THE DIRECTORY WHERE YOU SAVED THE IMAGES

ls <dir>/*-r*png | awk '{print $1" -1 -1 -1 -1"}' > input.txt

This command will produce a file with the following format:

'''
<dir>/1473188796.631450-r.png -1 -1 -1 -1
<dir>/1473188796.691370-r.png -1 -1 -1 -1
<dir>/1473188796.751127-r.png -1 -1 -1 -1
<dir>/1473188796.811682-r.png -1 -1 -1 -1
'''

3. Run the dataset_creator app using as parameter the input file produced in step (2), and the name of
the desired output file.

'''
./dataset_creator input.txt output.txt
'''

INSTRUCTIONS: 
- use 'n' to go to the next image
- use 'p' to go to the previous image
- use the mouse to select a rectangle around the object region.

As soon as you release the mouse button, the output file is updated with the coordinates of the new rectangle.

Ex.:

'''
<dir>/1473188796.631450-r.png 224 266 125 30
<dir>/1473188796.691370-r.png -1 -1 -1 -1
<dir>/1473188796.751127-r.png -1 -1 -1 -1
<dir>/1473188796.811682-r.png -1 -1 -1 -1
'''

