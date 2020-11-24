
### Voice setup
* cd catkin_ws/src/cr_ros_3/
* unzip the Ngrok zip using `unzip ngrok-stable-linux-amd64.zip`
* Authenticate Ngrok using `./ngrok authtoken <AUTH_TOKEN>`
* Run a gazebo world and launch the Alexa Skills console
* Run `roslaunch cr_ros_3 updated_voice.launch`
* Run `python3 src/voice_test_node.py`
* Speak to Alexa to try out a command!
