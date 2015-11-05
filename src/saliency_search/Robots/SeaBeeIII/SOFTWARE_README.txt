#################################################
SeaBuntu
#################################################

username: uscr
uscr/root password: 2cool!

#################################################
HOW TO MAKE A SHARED FOLDER FOR SEABUNTU/OSX:
-------------------------------------------------

To mount a shared folder under OSX do the following:
make a folder in SeaBuntu in /mnt called 'osx'
make a folder in OSX wherever you want the shared folder to be
shut down SeaBuntu, in virtual box main window click on SeaBuntu, but don't hit start yet. scroll down to 'Shared Folders', click it, then click green plus to add a shared folder, and tell it where the folder you just made in OSX is.
start up SeaBuntu
type the following in a terminal
sudo mount -t vboxsf Shared /mnt/osx
now you have a shared folder between osx and ubuntu

#################################################
WHERE OUR STUFF IS
-------------------------------------------------

Main Files we need to modify are the ones in:
/saliency/src/Robots/SeaBeeIII/
and also
/saliency/src/Ice/RobotSimEvents.ice

#################################################
HOW TO USE SVN:
-------------------------------------------------

(1) To update to the latest code, go to ~/saliency and run:
svn up
-or-
svn update

(2) To save changes you have made to the SVN, go to ~/saliency and run:
svn commit -m "detailed message about what you changed here"
-or-
svn commit -m <enter>
this will open a text editer so you can write you edit message more cleanly
simply save from the text editer and then hit enter

#################################################
ARCHITECTURE 101
-------------------------------------------------

(1) Message that are sent between modules for SeaBeeIII are all inside the file:
~/saliency/src/Ice/RobotSimEvents.ice

(2) Each task SeaBeeIII will do has its own module, aka vision module, movement module etc. Each module is in ~/saliency/src/Robots/SeaBeeIII and has the following 3 parts:
<module_name>.C file
<module_name>.H header file
test-<module_name>

(3) The makefile is located at ~/saliency and is called depoptions.in

#################################################
HOW TO COMPILE AND RUN A MODULE:
-------------------------------------------------

(1) if you edited RobotBrainComponent.ice, you must convert it to cpp before you compile, so go to ~/saliency/src/Ice and run:
slice2cpp RobotSimEvents.ice -I../

(2) from ~/saliency, run:
make bin/test-<module name>

(3) then, from saliency/src/Simulation, run:
icebox --Ice.Config=IceStorm.icebox&
-or-
./startIce

(4) and lastly,
from ~/saliency, run:
./bin/test-<module_name>

#################################################
HOW TO MAKE A MODULE
-------------------------------------------------

(1) Add the new messages you will be using.
In RobotSimEvents.ice (home/uscr/saliency/src/Ice/), append your message class at the end of the messages but before
the rest of the code in the file. Every time you update RobotSimEvents.ice you need to run the "slice2cpp RobotSimEvents.ice -I../" command to create a .cpp version.

(2) Create a module. It is best to make it in the SeabeeIII directory to
keep modules together. (home/uscr/saliency/src/Robots/SeabeeIII/) Copy/paste a sample module making sure to replace the names of the
classes/Messages/Topics appropriately. Do that for the associated files such as (*.H, *.C, test-*). Remember to declare yourself as a publisher/subsciber depending on the messages you are waiting for/will send...
See the MovementController files for a working example, working in
conjunction with the SeaBeeInjector files.

(3) Update the pseudo-makefile, depoptions.in (home/uscr/saliency/)
Look for a Seabee section and append your module using similar syntax to be
compiled.

(4) Compile and test your module, and then commit it to the svn once its working.

#################################################
