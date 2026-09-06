# Robot
A robot I am building that will follow you around

## Modes
### Auto Mode
**Overview**  
In auto mode the robot is designed to track the position of the transmitter and go to it.  
The DWM3000 can provide highly accurate and precise measurements when done correctly. The  
method I used for finding the distance between the robot (receiver) and the transmitter is known as  
double-sided two-way ranging. After calculating the distance I apply a lowpass fir filter to it using  
a hamming window with 31 taps. 

**How double-sided two-way ranging works**  
6 timestamps are measured: t1 (measured when the transmitter  
sends the first frame), t2 (measured when the receiver receives that first frame), t3  
(measured when the receiver sends its first response), t4 (measured when the transmitter  
receives the receiver's first response), t5 (measured when the transmitter sends the second  
frame), and t6 (measured when the receiver receives that second frame).  

From the 6 timestamps, 4 time values are calculated. The round trip time for the first frame,  
the round trip time for the second frame, the reply time for the first frame, and the reply  
time for the second frame. The time of flight (tof) then calculated as:  
(round trip 1 * round trip 2 - reply 1 * reply 2) / (round trip 1 + round trip 2 + reply 1 + reply 2)  

<details> 
  <summary> For the proof on why the tof equation works click here </summary>

$$
\begin{aligned}
\text{DS-TWR Time-of-Flight Derivation} \\[1em]
\text{Definitions:} & \\
R_1 &= 2T + D_1 \\
R_2 &= 2T + D_2 \\[1.5em]
\text{1. Numerator Expansion:} & \\
\mathrm{tof\_num} &= R_1 R_2 - D_1 D_2 \\
&= (2T + D_1)(2T + D_2) - D_1 D_2 \\
&= 4T^2 + 2T D_2 + 2T D_1 + D_1 D_2 - D_1 D_2 \\
&= 4T^2 + 2T(D_1 + D_2) \\
&= 2T(2T + D_1 + D_2) \\[1.5em]
\text{2. Denominator Expansion:} & \\
\mathrm{tof\_denom} &= R_1 + R_2 + D_1 + D_2 \\
&= (2T + D_1) + (2T + D_2) + D_1 + D_2 \\
&= 4T + 2D_1 + 2D_2 \\
&= 2(2T + D_1 + D_2) \\[1.5em]
\text{3. Final Division and Cancellation:} & \\
\text{tof} &= \frac{\mathrm{tof\_num}}{\mathrm{tof\_denom}} \\[0.5em]
&= \frac{2T(2T + D_1 + D_2)}{2(2T + D_1 + D_2)} \\[0.5em]
&= \frac{2T}{2} \\[0.5em]
&= T
\end{aligned}
$$

</details>

We then multiply the time of flight by 15.65E-12 to get from DWT system time to seconds.  
Finally, to calculate the distance (in cm) the tof in seconds is multiplied by 100 times  
times the speed of light.  

### Manual Mode
**Overview**  
In manual mode the user has full control over the robot by using the joystick on the transmitter.  
The RP2040-Zero features a 12 bit adc, but I limit the range to 0 to 800 and then shift it by  
540 to make the rest position 0, the low position -240, and the highest position +260, in both  
x and y directions. I implemented a software deadzone of 50 adc values to prevent unwanted drifting  
when in manual mode.  
  
**How the direction and speed is decided**  
I compare the x and y adc value. If x > y then we want to turn the robot, otherwise we want to move  
forward or backward. If we are turning and x > 0 we want to go left, otherwise we go right. If we are  
not turning and y > 0 we want to go forwards, otherwise we go backwards. The amount of steps we want the  
robot to take is calculated by mapping the adc value to a steps value, using -240 and +260 as the adc range    
and 10 and 100 as the step range. This information is then sent to the robot in 3 bytes. (0xAB to indicate  
manual mode, a direction byte, and the number of steps as the final byte).  

<img width="975" height="1158" alt="image" src="https://github.com/user-attachments/assets/d66f29f9-ed1f-4a82-8224-452b13564fa8" />
