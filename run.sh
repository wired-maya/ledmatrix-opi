#!/bin/sh
gcc -o ledmatrix main.c -l wiringPi -Ofast -lm -ldl -Wall
sudo nice -n -20 ./ledmatrix