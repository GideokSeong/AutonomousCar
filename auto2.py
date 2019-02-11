#!/usr/bin/env python
import RPi.GPIO as GPIO
import video_dir
import car_dir
import motor
import lane
from socket import *
from time import ctime          # Import necessary modules   


ctrl_cmd = ['forward', 'backward', 'left', 'right', 'stop', 'read cpu_temp', 'home', 'distance', 'x+', 'x-', 'y+', 'y-', 'xy_home']

busnum = 1          # Edit busnum to 0, if you uses Raspberry Pi 1 or 0

video_dir.setup(busnum=busnum)
car_dir.setup(busnum=busnum)
motor.setup(busnum=busnum)     # Initialize the Raspberry Pi GPIO connected to the DC motor. 
video_dir.home_x_y()
car_dir.home()

spd = 50
tmp = 'speed'
#global spd
#spd = speed.get()
data = tmp + str(spd)  # Change the integers into strings and combine them with the string 'speed'. 
print('sendData = %s' % data)

numLen = len(data) - len('speed')
if numLen == 1 or numLen == 2 or numLen == 3:
    tmp = data[-numLen:]
    print 'tmp(str) = %s' % tmp
    spd = int(tmp)
    print 'spd(int) = %d' % spd
    if spd < 24:
        spd = 24
    motor.setSpeed(spd)


while True:
    print('Succeed to connect...')
    data = raw_input("type among: forward, backward, left, right, stop, read cpu_temp, home, distance, x+, x-, y+, y-, xy_home")
	# Waiting for connection. Once receiving a connection, the function accept() returns a separate 
	# client socket for the subsequent communication. By default, the function accept() is a blocking 
	# one, which means it is suspended before the connection comes.
#	 tcpCliSock, addr = tcpSerSock.accept() 
	#print '...connected from :', addr     # Print the IP address of the client connected with the server.
    
    while True:
            
        #data = ''
    #    print("The command you input is " + data)
    #		data = tcpCliSock.recv(BUFSIZ)    # Receive data sent from the client. 
                    # Analyze the command received and control the car accordingly.
        if not data:
            print('Input cannot be recognized')
            break
        if data == ctrl_cmd[0]:
            print('motor moving forward')
            motor.forward()
        elif data == ctrl_cmd[1]:
            print('recv backward cmd')
            motor.backward()
        elif data == ctrl_cmd[2]:
            print('recv left cmd')
            car_dir.turn_left()
        elif data == ctrl_cmd[3]:
            print('recv right cmd')
            car_dir.turn_right()
        elif data == ctrl_cmd[6]:
            print('recv home cmd')
            car_dir.home()
        elif data == ctrl_cmd[4]:
            print('recv stop cmd')
            motor.ctrl(0)
        elif data[0:5] == 'speed':     # Change the speed
            print(data)
            numLen = len(data) - len('speed')
            if numLen == 1 or numLen == 2 or numLen == 3:
                tmp = data[-numLen:]
                print('tmp(str) = %s' % tmp)
                spd = int(tmp)
                print('spd(int) = %d' % spd)
                if spd < 24:
                    spd = 24
                motor.setSpeed(spd)
        elif data[0:5] == 'turn=':	#Turning Angle
            print('data =', data)
            angle = data.split('=')[1]
            try:
                angle = int(angle)
                car_dir.turn(angle)
            except:
                print('Error: angle =', angle)
        elif data[0:8] == 'forward=':
            print('data =', data)
            spd = data[8:]
            try:
                spd = int(spd)
                motor.forward(spd)
            except:
                print('Error speed =', spd)
        elif data[0:9] == 'backward=':
            
            print('data =', data)
            spd = data.split('=')[1]
            try:
                spd = int(spd)
                motor.backward(spd)
            except:
                print('ERROR, speed =', spd)

        else:
            print('Command Error! Cannot recognize command: ' + data)

#tcpSerSock.close()


