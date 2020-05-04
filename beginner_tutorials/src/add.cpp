/*
 * add.cpp

 *
 *  Created on: Mar 29, 2016
 *      Author: icgel
 */
#include "ros/ros.h"						//for sync ros
#include "add.h"

int add(int a, int b)
{
	int sum;
	sum = a*b;
	return sum;
}



