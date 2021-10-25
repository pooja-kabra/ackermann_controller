[![Build Status](https://app.travis-ci.com/markosej11/ackermann_controller.svg?branch=main)](https://app.travis-ci.com/markosej11/ackermann_controller)
[![Coverage Status](https://coveralls.io/repos/github/markosej11/ackermann_controller/badge.svg?branch=main)](https://coveralls.io/github/markosej11/ackermann_controller?branch=main)


Overview and purpose of the project (what does it do? Main Features? This should be a write-up of several paragraphs like a short report). Include results/performance examples.

# Ackermann Steering Controller

The starting point for explaining the Ackermann steering geometry is obvious: while on a bend, the outer front wheel traces a wider curve than the inner wheel (which has a narrower curve). A turning kart must have a rotation centre around which to do so, and around which the front wheels can rotate.
If the front wheels are perfectly parallel during the bend, no rotation centre would be created and the front wheels would slide, generating friction between tread and asphalt, tyre wear and loss of performance.

Ackermann’s steering geometry serves to offset the different direction covered by the front wheels along the curved trajectory to avoid this happening.
How? By causing the front wheels to rotate in a non-linear direction when turning the steering wheel. In doing so, a theoretical point is created (“theoretical” because, with the slipping of a 4-wheeled kart, the point actually varies) in which the 2 axes of the front wheels and the axis of the rear axle intersect.

Among other things, the bend geometry of a kart’s steering will lower the inner front wheel and lift the outer front wheel, with the transfer of much of the load to the front. The grip on the front is accentuated and the wheel travel direction is even more decisive.

In this project, we implement an Ackerman Steering controller for a robot. We assume that it is four-wheeled and rear drive robot. The task is to make the robot attain the desired heading and speed in a fixed World Frame. 

System Input: goal heading(degree, world frame), goal speed(degree, world frame)
System Output: inner wheel angular speed(rotations per sec), outer wheel angular speed(rps), inner wheel angle(degree, robot frame), outer wheel angle(degree, robot frame)
               
(For inner wheel angle and outer wheel angle, by robot frame, we mean that they are measured with respect to the robot's longitudinal axis)























































# Authors
Pooja Kabra (Graduate Student in Robotics at the University of Maryland College Park, May 2022), 
Markose Jacob (Graduate Student in Robotics at the University of Maryland College Park, Dec 2021)

# Shareable link to sprint planning notes and review Google Doc
https://docs.google.com/document/d/1BkXeVk6V7ksHjORvei8yQkGbdUtxJc3uc8dqAFlbF6g/edit

# Sections for (stubs in Phase I, completed by end of Phase II):

# Operation/run/test/demo steps

# Dependencies (and how to install if not included in the repository)

# Known issues/bugs

# API and other developer documentation (e.g. parameters and their definitions and default values)


# How to build
1. On your local computer, at the desired location, open a terminal and clone the repository by running
	$ git clone --recursive https://github.com/pooja-kabra/ackermann_controller/tree/second_phase

2. $ cd <path to repository> i.e. (../ackermann_controller/)
	$ mkdir build
 	$ cd build

3. In the same terminal(in ../ackermann_controller/build/), run 
	$ cmake ../
   (Every time you add new files to app, you should edit ../ackermann_controller/app/CMakeLists.txt to include them and run $ cmake ../)

4. In the same terminal, run
	$ make
	
# How to run the demo
1. After finishing build, run(in ../ackermann_controller/build/)
	$ ./app/shell-app

2. Enter desired goal heading and goal speed values.

# How to run tests
	$ ./test/cpp-test
# How to generate Doxygen documentation
The repository contains documentation. Although, if you would still like to generate it, follow the instructions below:

1. Install doxygen using below commands
	$ sudo apt-get install doxygen
	$ sudo apt-get install doxygen-gui

2. After installation run following command to open the doxywizard.
	$ doxywizard
   Here, you can fill in the details as required and set the source code folder to the repository as well. Create a new folder in the repository and select that as the destination directory. Add paths to include and src folders and then proceed with the default settings and generate the documentation.

# Product Backlog Google spreadsheet:
https://docs.google.com/spreadsheets/d/1yhOUowKENOIwfs4re1dwxyGSYlhCP-KEqHWIA955CD0/edit#gid=0

Copyright <2021> <POOJA KABRA> <MARKOSE JACOB>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
