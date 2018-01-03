# COMPGX01 CW3

## Installation
  * Clone this repo into the src folder of the workspace
  * Download the dependencies zip file from moodle
  * Unzip the dependencies into the src folder of your ros workspace
  * Build and source your ros workspace
  
## Launch Fils
  * q1_grasping.launch
  * q2_vision_grasping.launch
  * q3_dynamics.launch

## Usage
Each question has it's own launch file, with a number of arguments.
All the launch files have the following arguments:
  * gui :{true, false} - toggles rviz 
  * gz_gui :{true, false} - toggles the gazebo gui
  * scene :{1, 2} - changes the loaded scene

## Example
For questions 1 and 2 an example script is provided in scripts/example_script.py

## Hints
### Grasp controller
There are a number of ways to implement a grasp controller a simple version would be to manually move the robot into pregrasp and grasp poses recorded relative to the object. However the controller may struggle with unseen object poses. Alternatively you could use knowlegde of the geometry of the object to plan the grasp poses for contact with the object.
