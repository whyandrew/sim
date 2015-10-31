#!/bin/bash

echo $1    

#2 Thrusters      
if [[ $2 -eq 1 ]] 
then
	for var1 in 1 2 3
	do
	   for var2 in 1 2 3
	   do
	      if [ $var1 -ne $var2 -a $var1 -lt $var2 ]
	      then
	       	echo "$var1 $var2"
	      	./Lander_Control $1 3 $var1 $var2
	      	echo "-----------------"
	      fi
	   done
	done

#2 Sensors   
elif [[ $2 -eq 2 ]] 
then
	for var1 in 4 5 6 7 8 9
	do
	   for var2 in 4 5 6 7 8 9
	   do
	      if [ $var1 -ne $var2 -a $var1 -lt $var2 ]
	      then
	       	echo "$var1 $var2"
	      	./Lander_Control $1 3 $var1 $var2
	      	echo "-----------------"
	      fi
	   done
	done

#1 Thruster 1 Sensor
elif [[ $2 -eq 3 ]] 
then
	for var1 in 1 2 3
	do
	   for var2 in 4 5 6 7 8 9
	   do
		   	echo "$var1 $var2"
		  	./Lander_Control $1 3 $var1 $var2
		  	echo "-----------------"
	   done
	done
fi