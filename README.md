# Dodging-drone

# Team-Members:
1. Riyanshi Mandawara
2. Ritik Payak
3. Raviraj Bhosale

# Concept:
This project aims to design a dodging drone to determine the obstacle using ultrasonic sensor and servo. The avoiding system consists of two main parts which consist of detecting obstacle within a safe operation range and determining a safe flightpath in avoiding the obstacle.


The detected obstacle will be used as a data to
construct an ellipsoid restricted zone to determine the avoidance path.
Figure 1 illustrate the ellipsoid restricted zone which will be used to generate a contact point as the
guidance. It use an optimal path between the heading angle, flight velocity, ellipsoid shape, and
clearance [5]. The method is in default used for a fixed wing type of aircraft. Since this research uses
quadcopter, some adjustment to the flight manoeuvre and a simpler contact point algorithm is used.

![image](https://user-images.githubusercontent.com/99550382/178240148-9b49647a-c9d6-4b79-a2ce-34185ab205f4.png)




# Hardware

Designed system should be able to avoid the obstacle using on board processing. Flight between
waypoints will be provided by the flight controller and the avoidance will be performed by
microcontroller. All the hardware mentioned on previous section combined as shown in Figure 3.

2.1. Arduino

Arduino UNO is a microcontroller using ATMmega328. It provides 14 digital input/output pins
(including 6 PWM output pins), 6 analog pins, USB connection, and a power connector [6]. It will
be used to receive information from sensor, communicate with ground station, and control another
device to perform obstacle avoidance. Arduino needs to be connected to telemetry 2 of pixhawk controller.

2.2. Pixhawk

Pixhawk is an open source autopilot or flight controller developed by Computer Vision and
Geometry Lab of ETH Zurich and Autonomous System Lab with collaboration from other
researchers. Developed from 3DRobotics and ArduPilot Group, Pixhawk mainly used for hobby,
academic, and industrial purposes.
Pixhawk comes with several features such as: 168 MHz Cortex M4F CPU with 256 kb RAM
and 2 MB Flash; 3 axis accelerometer, gyroscope, magnetometer, and barometer; Integrated
backup, override, and failsafe processor with mixing; and microSD slot, 5 UARTs, CAN, I2C, etc
On this research, quadcopter firmware is used to match the platform. User input comes with a
pulse-position modulation (PPM) signal and the output is pulse-width modulation (PWM) signals.
Since Arduino as the microcontroller only provides PWM signal output, PWM to PPM converter
module is used to match the communication.

2.3. Ultrasonic Sensor

HRLV-MaxSonar-EZ Products is an ultrasonic sensor provided by MaxBotix. The sensor capable
of 1 mm resolution with maximum range of 5000 mm. Object closer than 30 cm will be counted as30 cm. Another specification is the output signal comes in various form such as analog voltage,
RS232 or TTL serial, and pulse-width. The sensor operates from 2.5V to 5.5V. With all the
specification and its small and light weight criteria, the sensor is suitable for research application
yet comes within affordable price.

![image](https://user-images.githubusercontent.com/99550382/178240256-6874e67b-f8f3-4786-9705-af794967e7d5.png)

Position of sensors:
![image](https://user-images.githubusercontent.com/99550382/178241479-276a6491-b628-41af-a5e7-db92c9725752.png)



2.4. Servo Gimbal

Servo gimbal used in this research is a 2-axis gimbal, in pan and tilt axis. Pan axis used in obstacle
detection by using it to rotate ultrasonic sensor within defined angle. The tilt axis function as a
stabilizer so the ultrasonic will always pointed frontward.

2.5. Telemetry

Several data such as distance to obstacle, time stamp, mode, and user input need to be recorded so
further analysis can be done. Telemetry is the easiest way to record the data. All those variables
inside microcontroller will be sent to laptop via Arduino IDE Serial Monitor. The data stored in
excel file and being processed later.

2.6. Airframe

Quadcopter is chosen as the airframe. First consideration is this type of airframe have the ability to
hover so lower computing ability of microcontroller is acceptable. Another one is the control
requirement is relatively simple as user command directly turns into a non-coupling movement of
the quadcopter.

#Construction :

![image](https://user-images.githubusercontent.com/99550382/178256430-ec793624-4b55-494a-9d85-7ef92d991c0d.png)


![image](https://user-images.githubusercontent.com/99550382/178241323-714d0416-d09c-4cd7-bd29-74e0a8e94297.png)


![image](https://user-images.githubusercontent.com/99550382/178241387-a2969205-6ab7-41bd-9967-44d985220a20.png)



3. Microcontroller Algorithm

3.1. Obstacle Avoiding Algorithm

The main mission of this algorithm is to enable the quadcopter fly between predefined waypoints
and to avoid obstacle between those waypoint if it exist. There are 5 mode for the algorithm from
the quadcopter starts to take-off manually, fly autonomously, and avoiding the obstacle.
Several parameters are made as switch between modes. First parameter is clear distance,
defined as the minimum distance the quadcopter can fly autonomously. Another one is safe
distance as the minimum distance the quadcopter can start scanning. On the avoidance mode
where quadcopter aimed to fly forward and side at the same time, the distance to obstacle
should’ve decreased. So another parameter is defined that is clear to avoid distance, which the
quadcopter should not move any closer than this distance when avoiding the obstacle. Illustration
of the parameters is shown in Figure 4.

![image](https://user-images.githubusercontent.com/99550382/178240362-52130c74-2aea-447d-918c-2c3dc3e28b17.png)


Position Hold, or generally known as PosHold, is a flight mode which designed to maintain a
constant location, heading, and altitude by utilizing data from inertial measurement unit (IMU),
GPS module, and compass. This flight mode is preferable rather than others since it provide a
natural control sense by directly control vehicle’s lean angle based on pilot stick inputs.
To control horizontal position on the ground, roll and pitch control stich is utilized with default
maximum lean angle of 45 degrees. It means when the stick hit the maximum position on the
remote control, the command will be interpreted as attitude reference to keep lean angle at that
position. This maximum lean angle can be adjusted with ANGLE_MAX parameter using ground
station control.

Autonomous mode, known as Auto, is a flight mode which enabled the quadcopter to follow a
pre-programmed mission script which is stored in the flight controller by setting waypoints as the
navigation command and some other action command [ardupilot.org]. Since this flight mode
combine altitude hold (AltHold) and Loiter to control altitude and position, an attempt of those
flight mode should be done before utilizing this Auto. As well as the positon hold, Auto flight
mode require the same conditions applied before it can take-off.
Keep distance mode is a part of avoidance mode which prevent the quadcopter from hitting the
obstacle. This mode is activated whenever the quadcopter distance to obstacle is less than safe
distance. Autonomous mode will be switch to position hold mode and microcontroller will send
signal to flight controller to fly backwards based on measured distance to the obstacle. The rate of
quadcopter fly backwards decreases as the distance to obstacle increases.


![image](https://user-images.githubusercontent.com/99550382/178240437-b0366cda-5227-4252-926d-66a0a64dcfb8.png)


The avoiding process will be start from scanning the obstacle first during scanning mode.
Since ultrasonic sensor only capable of measuring distance for a single point, it will be rotated by
using servo. Rotating ultrasonic is in limited angle since reflection angle higher than 25 degree
will not be detected. These data later will be quantified into an elliptical shape obstacle and a
contact point will be resulted as path planning method result.
Figure 5 shows the relation between autonomous flight and the avoidance procedure. Keep
distance, scanning, and avoidance mode is represented by fly backwards block, start scanning, and
execute avoidance manoeuvre respectively. All these mode is depended on the sensor
measurement.
After the contact point is obtained, the quadcopter changes to avoiding mode. In this mode,
microcontroller send PWM signal to pitch and roll channel based on the planned path. Quadcopter
will change to waypoint following mode after the obstacle is already passed. It happens when the
measured distance is more than the clear distance. Since quadcopter needs to approach the
obstacle first, it is possible that the quadcopter fly too close to the obstacle. Therefore, the flying
mode will switch to keep distance.

3.2. Signal Processing Algorithm

The quadcopter used ultrasonic as a sensor to measure distance and the data will be used to trigger
the avoidance mode. During flight, the measurement experience noise which will disturb the flight
mode. Therefore, a filter should be applied to reduce those noise.
Filter used in this research is a moving median. This filter have a purpose to remove an outlier
in the data which can be interpreted as noise. For instance, there are six data to be computed its
median and there is one data which is relatively high than the others. This data will be sorted to the
end of the window set and never used as data [8].
𝑦[𝑛] = 𝑚𝑒𝑑𝑖𝑎𝑛(𝑥[𝑛], 𝑥[𝑛 − 1], 𝑥[𝑛 − 2], … , 𝑥[𝑛 − 𝑘])
y[n] = Chosen data
x[n] = Data from measurement
n = Current data
k = Size of filter window

Figure 6 shows the result of these median filter with 11 data window applied to a moving
quadcopter. It is noticed that some noise successfully reduced as in 18 seconds in sub-figure (a)
and in 12 second in sub-figure (b). This noise can cause the quadcopter directly switched to keep
distance mode but fortunately stay at autonomous mode because of the filter.

![image](https://user-images.githubusercontent.com/99550382/178240606-efb7cebb-92ac-4044-b80e-2827ca19b02b.png)




