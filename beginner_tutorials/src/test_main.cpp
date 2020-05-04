/*
 * main.cpp
 *
 *  Created on: Mar 29, 2016
 *      Author: icgel
 */
#include "ros/ros.h"						//for sync ros
#include <stdio.h>
#include "add.h"
#include <termios.h>
#include <sstream>
#include <iostream>


int main(int argc, char **argv)
{
	using namespace std;
	int a = 5;
	int b = 8;
	int c;
	ROS_INFO("Sum of [%d + %d]", a,b);
	c = add(a,b);
	ROS_INFO(" =  [%d]", c);
}



