# CWRUbotix Software Set Up and Tutorial

By the end of this “bootcamp” you will have a knowledge of Linux, ROS, our robot’s code, and be able to drive our robot around in our simulator! If you on the CWRUbotix team feel free to ask questions in the [nasa-rmc-software](https://app.slack.com/client/T168FACCC/C2JAJ8L66) Slack Channel. If you have stumbled across this tutorial and have questions feel free to email me at <rmc170@case.edu>.<br>

## Table of Contents
- [CWRUbotix Software Set Up and Tutorial](#cwrubotix-software-set-up-and-tutorial)
  - [Table of Contents](#table-of-contents)
  - [Part 1 Installing Linux](#part-1-installing-linux)
  - [Part 2 Installing ROS](#part-2-installing-ros)
  - [Part 3 Git](#part-3-git)
  - [Part 4 Software and Simulator](#part-4-software-and-simulator)
  - [Part 5 IDE](#part-5-ide)
  - [Part 6 Next Steps](#part-6-next-steps)

## Part 1 Installing Linux

Almost all robots use the operating system called Linux (imagine if your Windows robot decided to update without your permission). However, most of you probably have a Windows computer, luckily we can install a Virtual Machine to use Linux. These instructions should also work if you have a mac but it hasn’t been tested. If you already have a linux Virtual machine or better have a real Linux machine feel free to skip this step.

<ol>
    <li>
   
Click on the Windows or Mac icon under VMware Products from the [CWRU software center](https://softwarecenter.case.edu/index.php). Log in in the top right corner to view the products.
    </li>
    <li>
    Download VMWare Workstation Pro.
    </li>
    <li>
    We want to run Ubuntu 20.04 Focal Fossa in our VM. Download it [here](https://releases.ubuntu.com/20.04.3/).
    </li>
    <li>You can open a terminal as an application or by using the shortcut `Control+Alt+T`.
    </li>
    <li>
    [Here](https://www.pcsuggest.com/basic-linux-commands/) are a couple useful linux commands, try out the first 12 in a terminal. Don’t delete anything important!
    </li>
    <li>
    VIM is a text editor you can use when editing files in the command 
    line. The very general use is: type `vim <file>` to open a file. Press `a` to enter editing mode. Make your edits. Press `Esc` to exit editing mode. Type `:x` and press enter to save and quit. Or type `:q` to exit without saving. Try it out. Don’t worry, most code is written with a normal editor not Vim. 
    </li>
    <li>
    Tips: The VM is just another computer so you can install chrome and your favorite editor and whatever else you like. 
    </li>
</ol>

## Part 2 Installing ROS

ROS is the robot operating system. It’s not an operating system, at its core it's a framework for different parts of the robot to communicate with each other. 

<ol>
    <li>

Read [this](http://wiki.ros.org/ROS/Introduction) (very) basic intro to ROS, and you can also feel free to read this [more in depth intro](https://www.ros.org/core-components/). However you can also skip the more in depth intro because later you will learn by doing.
    </li>
    <li>
    [Here](http://wiki.ros.org/noetic/Installation/Ubuntu) are the instructions to install ROS Noetic onto Ubuntu 20.04. You may either follow the ROS guide or use the one provided below. To copy/paste text to a terminal, you can’t use Ctrl-C as is reserved for stopping running programs. Use Shift-Ctrl-C or V. 
    <ol>
        <!-- Line kept to better match ROS tutorial> <-->
        <li>
        Skip this step. Should already be setup.
        </li>
        <li>
        The sources.list tells Linux where to download new software packages from.<br>
        `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
        </li>
        <li> 
        Keys are a security measure to make sure the software packages haven’t been tampered with.<br>
        `sudo apt install curl # if you haven't already installed curl`<br>
        `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
        </li>
        <li>
        Do the Desktop install, not the Desktop-Full.<br>
        `sudo apt update` <br>
        `sudo apt install ros-noetic-desktop` <br>
        </li>
        <li>
        This command will add a command to your .bashrc that will setup the ros environment. The `source` command just runs a script. For those at home who don't know or don't remember .bashrc file is a file that runs everytime Linux starts up.<br> 
        `echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`<br>
        `source ~/.bashrc`
        </li>
        <li>
        These code lines will install some basic dependencies for ROS. We will install the specific dependencies later.<br>
        `sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`<br>
        `sudo rosdep init`<br>
        `rosdep update`<br>
        Test your ros build by running the command `roscore`.
        </li>
    </ol>
    <li>
    Go through the first 8 tutorials [here](http://wiki.ros.org/ROS/Tutorials). This will take a while but by the end you will have a good knowledge of ros. I think you can do tutorials 3 and 4 last as they aren’t as fun.
    </li>
    <li>
    [Additional tutorial](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber) about how to write code for nodes. (Not needed for next steps but very good knowledge to have)
    </li>
</ol>

## Part 3 Git 

Git is a version control system which allows you to track changes in your code and help multiple people work together on one project. Our code is stored on GitHub, a website for hosting git repositories. <br>

Follow [this](https://www.freecodecamp.org/news/what-is-git-and-how-to-use-it-c341b049ae61/) git tutorial. Note git should already be installed on your Linux VM. 
Create a [github](https://github.com/) account if you don’t have one.
Tell us your username so we can add you to our [organization](https://github.com/cwruRobotics).

## Part 4 Software and Simulator

At the end of this step you will be able to build and run our robot software in the simulation.

<ol>
    <li>

Create a catkin_ws in your home folder. The home folder is the ~ folder. Then create a `src` folder in the catkin_ws.<br>
    `mkdir -p ~/catkin_ws/src`
    </li>
    <li>
    Clone the [NASA-RMC-2020](https://github.com/cwruRobotics/NASA-RMC-2020) repository into `~/catkin_ws/src/`. <br>
    `cd ~/catkin_ws/src`<br>
    `git clone https://github.com/cwruRobotics/NASA-RMC-2020.git`
    </li>
    <li>
    Navigate to the `tools` folder inside of NASA-RMC-2020. <br>
    `cd NASA-RMC-2020/tools`<br>
    Run `install_ros_deps.sh` to install our ros dependencies. <br>
    Run `install_helper_programs.sh` to install helper programs (open these files if you’re curious what you’re installing).<br>
    `source install_ros_dep.sh`<br>
    `source insall_helper_programs`<br>
    `sudo apt upgrade`
    </li>
    <li>
    Build the code using `catkin build`. Note: need to be in the catkin_ws folder. `catkin build` is a nicer version of `catkin_make`.<br>
    `cd ~/catkin_ws`<br>
    `catkin build`
    </li>
    <li>
    Now that we have a workspace, it helps to automatically setup the workspace when we login. Add the commands `source ~/catkin_ws/devel/setup.bash` to your `~/.bashrc` file and then source the file.<br>
    `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`<br>
    `source ~/.bashrc`
    </li>
    <li>
    Our code has an option to store the data generated by our sensor for later review. To do this we must create the folder `USB_BAGGING/log` inside the home directory. Then we need to change default logging location of ROS to be within this folder.<Br>
    `mkdir -p ~/USB_BAGGING/log`<br>
    `echo "export ROS_LOG_DIR=~/USB_BAGGING/log" >> ~/.bashrc`<br>
    `source ~/.bashrc`
    </li>
    <li>
    For those new to linux anyhing in the .bashrc file will run automatically everytime you start up linux. It is a very basic way to automate some repetive things. To edit the .bashrc use `vim ~/.bashrc`. Note that because this folder starts with a `.` it is hidden to `ls` unless you do `ls -a`.
    </li>
    Gazebo needs to know the path to plugins built for it. Add the line `export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib/` to your `.bashrc`. It also needs to know where our arena models are. Add the line `export GAZEBO_MODEL_PATH=~/catkin_ws/src/NASA-RMC-2020/glenn_simulation/glenn_description/models/` to the  `.bashrc` as well.<br>
    `echo "export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib/" >> ~/.bashrc`<br>
    `echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/NASA-RMC-2020/glenn_simulation/glenn_description/models/" >> ~/.bashrc`<br>
    `source ~/.bashrc`
    </li>
    <li>
    Time to run the simulator!
    Open a terminal and run `roscore`.
    Open another terminal and run `roslaunch glenn_description glenn.launch`. (Set bags to true if you want to record data from the simulation).
    You should hopefully see our robot in the simulation. Try driving it around or playing with some of the windows that opened.<br>
    `roscore`<br>
    `roslaunch glenn_description glenn.launch`<br>
    </li>
</ol>

## Part 5 IDE

You are free to use whatever IDE you want as long as it can devolop for both C++ and Python. If you want to use VsCode on your VM we have file to install and setup.<br>
`source ~/catkin_ws/src/NASA-RMC-2020/tools/install_vscode.sh`<br>
To open VSCode either use `code -n ~/catkin_ws/src/NASA-RMC-2020/` or as an application.<br>
`code -n ~/catkin_ws/src/NASA-RMC-2020/`

## Part 6 Next Steps
Do [this](http://wiki.ros.org/rospy_tutorials/Tutorials/WritingPublisherSubscriber) tutorial on programming a ros node if you haven’t yet.
[Here’s](https://docs.google.com/document/d/1mGesu10JnqWqZkxbHqHrwBc_WiGuTWYFXLr6ffkSuD8/edit?usp=sharing) a document that explains the nodes currently on our robot. Run the robot simulator. Use `rosnode list` and `rosnode info <node_name>` to see what topics each node is publishing. Also use `rostopic list` and `rostopic info <topic_name>` to learn about the topics we use.
