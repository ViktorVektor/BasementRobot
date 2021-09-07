## 5-Axis Robotic Arm

![image](https://user-images.githubusercontent.com/41247872/132286744-f61dbbb3-6c37-4f1d-899b-ab8461038803.png)

A 5-axis robot arm projet with the purpose of understanding the 3D printing process and microcontroller basics. This project also included calculation with inverse kinematics both by hand and through software in Python.

### Project Goals

The goal of the project was to get my feet wet with 3D modelling, 3D printing, hands-on electrical work, matrix calculations, and Python programming. A robotic arm would touch on all of these aspects and has plenty of information online for how it operates. 

For the device, I set out a few requirements:
 - Lift at least 500 grams
 - Be mostly unconstrained in the motion of joints
 - Be mostly 3D printed
 - Operable though Python

### September 2020

I was initially inspired by robotic arms using servo motors, particularly ones sold online or available on sites like Amazon. (https://www.amazon.ca/SainSmart-Control-Palletizing-Arduino-MEGA2560/dp/B00UMOSQCI). These arms are controlled with an arduino, moved by servo motors.

![image](https://user-images.githubusercontent.com/41247872/132287775-6a344b85-e3f3-440d-b6de-1ad0c2425668.png)

### April 2021

Upon further reseasrch, I found that I preferred the motion achieved by industrial robots like the 6-axis Meca500 (https://youtu.be/lD2HQcxeNoA).

I started work on the mathematics behind how mechanical linkages would move, and eventually creating some software to help visualize how the robot arm would look. The end goal of this piece of software is to control the robot arm with Python, but the current state was more of an exercise in Python programming.

https://github.com/ViktorVektor/BasementRobot/blob/0fab545b65b477806c2f2ecbbd39898bb4239023/final%20main.py

In addition, I also explored communicating with an Arduino.

https://github.com/ViktorVektor/BasementRobot/blob/2452eb865f8f66fe7eb13dcaaa4190731e68ebba/gyro_servo_comm%20test.py

### May 2021

My research into the physical aspects of the arm started with the types of motor I would use. I knew I wanted more than 180 degrees of motion in a joint, so I started reading up on stepper motors. From my understanding, these types of motors would be used in places where precise position control and high torque is needed, while the weight of the motor itself not being a major consideration. My robot would not be operating in a speedy manner, so I decided on adopting the NEMA 17 and the 28BYJ-48 stepper motors. Both were very common and used the world over, so there was a lot of info and people talking about their experience with it online.

![image](https://user-images.githubusercontent.com/41247872/132289187-90b2d599-8861-4ba2-bb26-414e628a4d10.png) ![image](https://user-images.githubusercontent.com/41247872/132289226-345994f7-13b6-4321-8517-79b9121531ea.png)

Being that the 28BYJ-28 was a unipolar stepper, I converted it to a bipolar motor by modifying some wiring. This had the added bonus of increasing the available torque of the small motor, but increases the likelihood of overheating. I had also planned to use an Arduino Mega 2560 and a RAMPS 1.4 shield to control the robot. This controller combination was typically used in DIY 3D printers, but can serve the a similar purpose in powering my motors. The stepepr drivers I would be using was the A4988 Bipolar Stepper Driver, one also commonly used with the RAMPS board.

With the electricals determined, I turned my attention to the mechanical aspects of the robot. 

#### 3D Printing
