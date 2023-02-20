#!/bin/bash

rm /home/ros/.gazebo/gui.ini
echo "[geometry]" >> /home/ros/.gazebo/gui.ini
echo "x=72" >> /home/ros/.gazebo/gui.ini
echo "y=500" >> /home/ros/.gazebo/gui.ini
echo "width=883" >> /home/ros/.gazebo/gui.ini
echo "height=584" >> /home/ros/.gazebo/gui.ini