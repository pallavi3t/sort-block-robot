# Robot Block Sorter / Mover


## Project Overview

This is a robot simulation moving colored blocks onto a shelf. Blocks are supposed sorted by color on the shelf. 

Goal of this project was to use Gazebo simulator to move a UR3 robot to sort blocks of colors red, yellow, and green onto an object. The object chosen was a simulated shelf that the blocks would be placed in different locations of based on the color. Block colors are randomly generatored upon each spawn. 

Forward and inverse kinematics were also added to the robot to calculate the correct position to pick up the blocks and place it on the appropriate place on the shelf. The coordinates to pick up the blocks were returned by a camera. The camera also detected color which would change the location the block would be placed on the shelf. 

Video: https://youtu.be/ga_QIzeW6U8
