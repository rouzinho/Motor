# Motor Control in Cedar

This Cedar plugin allow the control of the Gummi Arm through a ROS Publisher

Everything you want to know about DFT -> https://dynamicfieldtheory.org/

Cedar is the C++ Framework implementing the concepts of DFT -> https://cedar.ini.rub.de/

Everything you need to know about ROS -> http://www.ros.org/

## Getting Started

The plugin is a widget reading outputs from a Neural Field and publishing the data to the motors of the Gummi.

You can define directly in the Qt Widget if you want to publish a Float64 on a topic or a JoinState message.

Of course you can adapt it to publish commands to any topics, but you might want to change the scale or format of the datas received from the Neural Field.

The code work for the 6.x version of Cedar.


### Prerequisites

You first need to install cedar by following the instructions here : https://cedar.ini.rub.de/tutorials/installing_and_running_cedar/

You can't use a precompiled version of Cedar to compile and run the plugin.

I suggest reading about how to create a plugin in Cedar first, it will greatly help to understand how it works : https://cedar.ini.rub.de/tutorials/writing_custom_code_for_cedar/

Install ROS : http://wiki.ros.org/ROS/Installation

The code was tested on ROS Kinetic Kame and ROS Melodic Morenia

**Warning**

ROS and Cedar are a bit to powerful to run on the same computer (if you have a big DFT model and a complex robot), so I recommend using 2 different computers.

**INSTALL YARP**

This last version of the plugin requires yarp (cedar built with yarp support - you don't have to do  it if you didn't include YARP when building Cedar)

https://www.yarp.it/install.html

If you don't need it, remove the find_package(YARP REQUIRED) in the cedarProject.cmake

### Installing

First clone the repository :

`https://github.com/rouzinho/Motor.git`

In the project.conf, change the CEDAR_HOME directory to your own :

`set(CEDAR_HOME "your_own_cedar_repository")`

Then create a build repository and prepare the environment for the compilation :

`mkdir build`

`cd build`

`cmake ..`

Finally start the compilation :

`make`

You should see the plugin under the name libMotor.so in the build/ repository

## Before Running the plugin

Start a ROS Init() Thread : https://github.com/rouzinho/RosInitCedar

## Run the plugin

Execute cedar and load it into cedar 

*Tools -> Manage plugins*

In the plugin Manager window, click on *add* and choose the plugin libMotor.so (located in build/). This one should appear in the window.

You can close the window. The plugin is loaded inside cedar and before loading it, make sure your ROS node is running.

You can now go back to the cedar main interface and click on the Utilities tab.

Drag the MotorHead widget into the architecture panel. Connect the output of a space to rate widget to the input of the MotorHead widget. The outputs of the Neural Field now drive the motor of your choice !


## Work in progress


The plugin is more like an artefact binding sensors to DFT.
Work in progress to use Qt elements to control the settings.



## Authors

Quentin Houbre - Tampere University

## License

This project is licensed under the BSD licence


