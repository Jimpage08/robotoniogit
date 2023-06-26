WRO Future Engineers is a competition that consists of two challenges:
1. Run1
2. Run2

During the first part (Run1) the robotic car, working autonomically, has to complete 3 timed laps around the track without any obstacles. The goal is to complete all the laps without touching any of the walls around the track, while also trying to finish the laps within a good amount of time.

For the second part (Run2) the same car has to complete 3 timed laps around the track with obstacles scattered randomly across the track. The obstacles, commonly referred to as pillars, can be either green or red. The robot has to avoid any red pillar it sees from the right, and any green pillar it sees from the left. The pillars can be placed in specific squares on the track which are annotated on the track surface itself. The positions of the pillars are defined during the competition, right before the second round of the competition begins.

In order to achieve all of this, we had to use sophisticated components, such as a DC motor for the movement of the robot, distance sensors placed on the left and the right sides of the robot, in order to check its distance from the walls, a gyroscope which is used for the robot's steering, and a camera which is used to detect the pillars and detect their colors. All of these are connected to the Jetson Nano Development Kit, which was developed by Nvidia. Specifically, Jetson Nano was especially useful for the video analysis achieved by the camera via computer vision.
