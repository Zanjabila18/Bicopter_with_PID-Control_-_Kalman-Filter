# Bicopter_with_PID-Control_-_Kalman-Filter
This project was carried out independently to fulfill the final project of the Digital Control System course by synthesizing PID control equations into embedded system. 

Ziegler Nichols 2 method is carried out by placing the system in a closed-loop state so that there is feedback through the sensor, in this plant system, namely the MPU6050 sensor. Furthermore, the response is made, that is, it must be in an oscillating state by increasing the Kp value and the Ti and Td parameters are turned off. After oscillation, the Kcr (Kp) and Pcr (oscillation period) values will be obtained from the oscillation response obtained.

![image](https://user-images.githubusercontent.com/93900620/206578748-e65c405e-3496-4c1e-9e65-5f974b2a7ad0.png)
