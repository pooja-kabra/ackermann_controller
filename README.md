[![Build Status](https://app.travis-ci.com/markosej11/ackermann_controller.svg?branch=main)](https://app.travis-ci.com/markosej11/ackermann_controller)
[![Coverage Status](https://coveralls.io/repos/github/markosej11/ackermann_controller/badge.svg?branch=main)](https://coveralls.io/github/markosej11/ackermann_controller?branch=main)
# ackermann_controller
This is a project we are working on for Acme robotics. We are working on building a program with steers the robot to a given goal heading and speed with the use of ackermann controller to reduce slippage.

# Product Backlog
below is the link to the google spreadsheet which contains our product backlog
https://docs.google.com/spreadsheets/d/1yhOUowKENOIwfs4re1dwxyGSYlhCP-KEqHWIA955CD0/edit#gid=0

# To Do
1) work on implementation

# Group members
1) Markose Jacob
2) Pooja Kabra

# Standard install via command-line:
```
git clone --recursive https://github.com/pooja-kabra/ackermann_controller/tree/phase1
cd <path to repository>
mkdir build
cd build
cmake ..
make
Run tests: ./test/cpp-test
Run program: ./app/shell-app ../config.txt
```

# Doxygen Documentation

Although the repository contains the documentation, if you'd still like to generate it then follow the instructions below.

Install doxygen using below commands
```
sudo apt-get install doxygen
sudo apt-get install doxygen-gui
```
After installation run following command to open the doxywizard wherein you can fill in the details as required and set the source code folder to the repository as well. Create a new folder in the repository and select that as the destination directory. Add paths to include and src folders and then proceed with the default settings and generate the documentation.
```
doxywizard
```
