<h1>Welcome to the NASA Software team!</h1>

By the end of this “bootcamp” you will have a knowledge of Linux, ROS, our robot’s code, and be able to drive our robot around in our brand-new simulator!

<h2>Part 1) Installing Linux</h2>

Almost all robots use the operating system called Linux (imagine if your Windows robot decided to update without your permission). However, most of you probably have a Windows computer, luckily we can install a Virtual Machine to use Linux. These instructions should also work if you have a mac but it hasn’t been tested. 

<ol>
    <li>

Click on the Windows or Mac icon under VMware Products from the [CWRU software center](https://softwarecenter.case.edu/index.php). Log in in the top right corner to view the products.</p></li>
    <li>
    Download VMWare Workstation Pro 
    </li>
    <li>
    We want to run Ubuntu 20.04 Focal Fossa in our VM. Download it here.
    Install Focal Fossa into your VM. Download it [here](https://releases.ubuntu.com/20.04.3/).
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

<h2>Part 2) Installing ROS</h2>

ROS is the robot operating system. It’s not an operating system, at its core it's a framework for different parts of the robot to communicate with each other. 

Read this (very) basic intro to ros, and you can also feel free to read this more in depth intro. However you can also skip the more in depth intro because later you will learn by doing.
Here are the instructions to install ROS Melodic onto Ubuntu 18.04. Please read the director's commentary below for each step. (To copy/paste text to a terminal, you can’t use Ctrl-C as is reserved for stopping running programs. Use Shift-Ctrl-C or V. 
1.1) Skip this step. Should already be setup.
1.2) The sources.list tells Linux where to download new software packages from
1.3) Keys are a security measure to make sure the software packages haven’t been tampered with
1.4) Do the Desktop install, not the Desktop-Full
1.5) This command will add a command to your .bashrc that will setup the ros environment. The `source` command just runs a script
1.6) Skip 1.6, only do 1.6.1 (although it’s not really necessary)
Test your ros build by running the command `roscore`
Go through the first 8 tutorials here. This will take a while but by the end you will have a good knowledge of ros. I think you can do tutorials 3 and 4 last as they aren’t as fun
Additional tutorial about how to write code for nodes. (Not needed for next steps but very good knowledge to have)

<h2> Part 3) Git </h2>

Git is a version control system which allows you to track changes in your code and help multiple people work together on one project. Our code is stored on GitHub, a website for hosting git repositories. 

Follow this git tutorial. Note git should already be installed in your Linux. 
Create a github account if you don’t have one.
Tell us your username so we can add you to our organization

<h2> Part 4) Our Software + Simulator </h2>

At the end of this step you will be able to build and run our robot software in the simulation.

Create a catkin_ws in your home folder. The home folder is the ~ folder. Then create a `src` folder in that folder. Clone the NASA-RMC-2020 repository into `~/catkin_ws/src/`. 
Navigate to the new NASA-RMC-2020 folder and run `git submodule update --init`
Navigate to the `tools` folder inside of NASA-RMC-2020. Run `source install_ros_deps.sh` to install our ros dependencies. Run `source install_helper_programs.sh` to install three nice helper programs (open these files if you’re curious what you’re installing)
Build the code using `catkin build`. Note: need to be in the catkin_ws folder. `catkin build` is a nicer version of `catkin_make`.
Now that we have a workspace, it helps to automatically setup the workspace when we login. Add the commands `source /opt/ros/melodic/setup.bash` (If it is not there already, you might have done this step in the ROS tutorials) and `source ~/catkin_ws/devel/setup.bash` to your `~/.bashrc` file.
To open the .bashrc file, do `vim .bashrc` while in the home (~) folder. Or do `vim ~/.bashrc` if you’re not in the home folder. Note that because this folder starts with a `.` it is hidden to `ls` unless you do `ls -a` 
Gazebo needs to know the path to plugins built for it. Add the line `export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib/` to your `.bashrc`. It also needs to know where our arena models are. Add the line `export GAZEBO_MODEL_PATH=~/catkin_ws/src/NASA-RMC-2020/glenn_simulation/glenn_description/models/` to the  `.bashrc` as well
Time to run the simulator!
Open a terminal and run `roscore`
Open another terminal and run `roslaunch glenn_description glenn.launch`
Open a third terminal and run `roslaunch glenn_launcher main.launch bags:=false sim:=true` (Set bags to true if you want to record data from the simulation)
You should hopefully see our robot in the simulation. Try driving it around or playing with some of the windows that opened

Next steps:
Do this tutorial on programming a ros node if you haven’t yet. 
Here’s a document that explains the nodes currently on our robot. Run the robot simulator. Use `rosnode list` and `rosnode info <node_name>` to see what topics each node is publishing. Also use `rostopic list` and `rostopic info <topic_name> to learn about the topics we use.









