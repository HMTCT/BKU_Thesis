#!/bin/bash

sudo chmod 777 /dev/ttyACM*
# python3 game_pad.py
rosrun gamepad game_pad_mqtt.py mqtt_sub_vr.py

