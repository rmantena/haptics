README:

Authors: Keith Dockstader and Rajiv Mantena
Haptics MEEN 7690 Final Project | Spring 2017 | University of Utah

------------ Summary ------------

This project was undertaken to demonstrate the advantage of using Wave Variables for Bilateral Teleoperation in presence of Time Delay. 
Hardware used were Two Novint Falcon devices. 
We have seen that using Wave Variables is much better than traditional Position Servo teleoperation, when there is a time delay, as the chances of the system going unstable are very very less. However, the force feedback is not perfect in presence of time delay. 
Watch the below video for a better understanding. 

[[Watch the video]](https://youtu.be/uJxbSLDXqgU)

Thanks! 


----------- Using this code -----------

The following sections outline the necessary steps for just running the executable file mirror.exe on your own computer and editing the code to generate a custom executable files.


///////////////////////////////////////////////////////////////////////////////////////////////////

Section I: Running the exectuable file

Go to http://www.forcedimension.com/download/sdk to download the SDK 3.6 with drivers corresponding to your device.

Run the executable file once it has completed downloading and be sure to install examples if you are interested in editing the code.

Now connect the two novint falcons to be teleoperated and run the exectuable file mirror.exe from whatever directory you like. This will begin traditional teleoperational control of the two devices.


Using current system:

Upon first running the program you may have to initialize the devices. To do so move the handle around through space for both devices until they begin to move by themselves.

Each time you run the program or switch method of teleoperation or alter the time delay the program will loop back and begin self-initialization. Do not interfere until its motion has stopped.

Press q to quit at any time or e to emergency stop the devices and reset.

The program initializes with traditional teleoperational control with no delay. To switch to traditional teloperational control with/without delay press y/t respectively. To switch to teleoperational control with wave variables press u.

The buttons to alter the gains and time delay are shown at all times.


///////////////////////////////////////////////////////////////////////////////////////////////////

Section II: Generating custom code

Complete Section I before beginning Section II.

Navigate to the Force Dimension installation directory (something like: C:\Program Files\Force Dimension\sdk-3.6.0) and navigate to examples/CLI/mirror.

Open the mirror project and compile.

This should begin an executable that runs normal teleoperation alone.

To generate custom code, paste in the code from hapticsProject.cpp to the source file mirror.cpp in the mirror project and begin changing as necessary.

The relavent sections of code are the following:

	/*-------------------------------------------------------------------------------------------------------*
	*		Bilateral Teleoperation (Position Servo) without time delay | Simplest code possible	 *
	*--------------------------------------------------------------------------------------------------------*/

	/*-------------------------------------------------------------------------------------------------------*
	*		                 Bilateral Teleoperation (Position Servo) with time delay 		 *
	*--------------------------------------------------------------------------------------------------------*/

        /*-------------------------------------------------------------------------------------------------------*
	*		               Bilateral Teleoperation using Wave Variables with time delay  		 *
	*--------------------------------------------------------------------------------------------------------*/

These sections perform the teleoperation of both devices.

For testing it was useful to put an automatic shutdown in the case of the haptic device positions getting outside of a perscribed position. This is no longer in place but can easily be implemented if desired.
