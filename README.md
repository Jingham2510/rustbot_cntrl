# rustbot_cntrl

Rust based cmd line controller for an ABB_IRB6400 situated in the TerraRobotics Laboratory at the Civil Engineering Department in Cambridge.

Currently the controller is also capabale of communicating with an Intel Realsense depth camera, which provides depth measurements of the terrain.

In relatime, this osftware is able to control and acquire data from the robot and measure the terrain depths.
Steps are being taken to turn this into a full-loop control system.

The target robot is running the IRB6400 RAPID code found here - https://github.com/Jingham2510/IRB_6400_TCP
