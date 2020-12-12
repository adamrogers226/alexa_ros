### Voice setup
* cd catkin_ws/src/cr_ros_3/
* unzip the Ngrok zip using `unzip ngrok-stable-linux-amd64.zip` (only once)
* Authenticate Ngrok using `./ngrok authtoken <AUTH_TOKEN>`
* Run a gazebo world and launch the Alexa Skills console
* Open a new terminal and run `./ngrok_launch.sh`
* Open a new terminal and run `python minimal_voice_webhook.py`
* Open a new terminal and run `python voice_test_node.py`
* Open a new terminal and run `python command_runner.py`
* Speak to Alexa to try out a command!

### Introduction

The original goal for this project was to run SLAM/AMCL on a robot in a simulated environment by giving it commands via a connection with Alexa. The main functionality I wanted to achieve was for the robot to explore the area and create a map while marking down locations for it to travel back to. When mapping completed, the goal was for the robot to be able to navigate to each of these saved locations. 

### Implementation

Before I could start implementing each command I wanted to give to the robot, I had to establish a connection with Alexa so I could feed in commands. This is why I forked over from the campus rover repo where the voice connection was already set up. In a nutshell, ngrok is used to create an IP address that Alexa can communicate with. Then, the minimal voice webhook script is what takes in the request from Alexa and publishes to a ros topic that can be easily interpreted. The voice test node subscribes to this topic and the majority of the implementation is in there.

After creating the custom commands I wanted the robot to perform (e.g marking a location, start mapping the area, etc.), I had to implement these in voice test node. For marking locations, I simply had a subscriber listening to the odom topic. Every time a new odom would be received, I saved it into a variable. When the user tells Alexa to mark a location, the node saves the current odom pose into a dictionary with the key being the number the user specified and the value being the pose. 

For some tasks I needed to complete, launch files had to be run. So, I had to do some research on how to publish commands to a terminal from a Python script. I found the `subprocess` module very useful for this. However, I did not want these commands to be run in the same terminal that I had been running the voice test node in, so I created a new node called `command_runner.py`. This node subscribes to a new topic called `/cmds_to_run` which takes String messages. Whenever the user tells the robot to perform a command that needs to publish to a terminal, I publish that command as a String which is then executed by the command runner node. The node is also capable of stopping commands currently being run by terminating all commands which is necessary for when the user wants to stop mapping and begin navigating.

### Reflection

This project was difficult at many times because of the obscure errors that would arise from the mapping and navigation. Since we didn't go very in depth with these aspects in class, it was hard to know exactly what to do in each scenario. Another challenge I had in the beginning was creating custom commands with Alexa and just the whole interaction between Alexa and Python. However, once I was able to figure these things out, I was super happy with my result. My goals were met, and although I wish I could have maybe gone a bit further, I'm satisfied that I was able to accomplish what I set out to do. 
