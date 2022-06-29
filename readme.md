# Self Balancing Robot using STM32

## Abstract
Recently quite a few work has been finished in the self balancing of gadgets. The idea of self balancing started with the balancing of inverted pendulum. This concept prolonged to design of aircrafts as well. in this venture, we have designed a small model of self balancing robotic using the PID(Proportional, quintessential, spinoff) set of rules. when you consider that then, this technique is the new face of the economic technique manipulate systems. This record reviews the methods worried in self balancing of items. This task became performed as a semester challenge to recognize the correlation of PID on the efficiency of various commercial procedures. right here we focus
handiest at presenting a short evaluation at the eﬀectiveness and application of the PID manipulate. This paper has been developed through offering a short advent to manipulate structures and associated terminologies, with addition to the motivations for the assignment. Experimentation and observations were taken, deserves and demerits described with ending on the future
enhancements. A version of self balancing robot became developed to recognize the eﬀectiveness of PID in the global of manipulate gadget. Going thru some rigorous checks and experiments, the merits and demerits of the PID control device had been determined. It changed into observed that in spite of many advantages of PID manipulate over beyond methods, nonetheless this device requires quite a few upgrades. it's miles was hoping that the reader gets a very good expertise of the
importance of self balancing, the eﬀectiveness and deficiencies of PID manipulate




## Introduction:
With the arrival of computer systems and the industrialization of approaches, all through man‘s records, there has continually been research to expand ways to refine strategies and extra importantly, to manipulate them using machines autonomously. The cause being to lessen man‘s
involvement in those techniques, thereby decreasing the mistake in those strategies. consequently, the sphere of ”manipulate system Engineering” become evolved. manipulate machine Engineering may be defined as the usage of various techniques to govern the operating of a process or upkeep of a constant and preferred environment, be it guide or automatic.


A easy example could be of controlling the temperature in a room. guide manipulate approach the presence of someone at a website who exams the prevailing situations (sensor), compares it with the favored cost (processing) and takes appropriate movement to gain the desired cost (actuator). The trouble with this technique is that it isn't always very dependable as someone is liable to blunders or negligence in his paintings. additionally, every other hassle is that the rate of the system initiated by way of the actuator isn't usually uniform, that means sometimes it may occur quicker than required or now and again it may be sluggish. the solution of this hassle turned into to apply a micro-controller to control the system. The micro-controller is programmed to control the method, in step with given specifications, related in a circuit , fed the favoured conditions and thereby controls the process to hold the favored condition. The advantage of this process is that no human intervention is needed on this process. also, the charge of the procedure is uniform
fundamental manipulate machine. The microcontroller is on the heart of any control machine. it's for a completely critical aspect therefore, its desire of choice must be made carefully based totally on the requirements of the device. The micro-controller gets an input from the user. This enter defines the preferred condition of the gadget. The micro-controller also gets a comments enter from the sensor. This sensor is hooked up to the output of the system, the statistics of that is fed back to the input. The microprocessor, based on its programming, performs diverse calculations and gives an output to the actuator. The actuator, based totally at the output, controls the plant to try to preserve the ones situations. An example can be a motor driver driving a motor wherein the motor driver is the actuator and the motor is the plant. The motor, accordingly rotates at a given velocity. The sensor connected reads the circumstance of the plant at the existing time and feeds it again to the micro-controller. The micro-controller again compares, makes calculations and consequently, the cycle repeats itself. This process is repetitive and endless whereby the micro-controller continues the desired conditions

## Methodology
We have used 3 ports of the stm32
1)Port B for communicating with the MPU6050 sensor using I2C
2)	Port A connected to L289N motor driver for controlling the
direction of the motors
3)	Port D connected to L289N motor driver for controlling the speed of the motor

## PORT B : (for the I2C )
The pins PB6 and PB7 are used for the I2C communication
between the stm32 and the MPU6050.
PB6 is connected to the SCL and PB7 to the SDA pin of the MPU6050.

## Pin Configuration:
The pins PB6 and PB7 are set to
•	Alternate function mode as they are used for I2C.
•	Open drain and pull up mode
•	High speed mode

## Port A :
The pins A1,A2,A3 and A4 are used
Connected to the L289N IC for controlling the direction of the
motor.
## PA1-> IN1	PA3->IN3 PA2->IN2	PA4->IN4

## Configurations :
1)General purpose output mode
2)Pull down mode
3)High speed
## Port D:
PD12,PD13,PD14 and PD15 pins are used as PWM output pins connected to the L289N enable pins 

## Configuration:
* Alternate function mode for PWM
* Pull up and High speed mode
* PWM Configurations
* Timer 4 is used for generating the PWMs
* The system clock frequency of 84MHz is prescaled to drive the
* Timer 4 at 21MHz frequency
* The frequency of the PWM is kept 20kHz by keeping the ARR value as 1050

### Total 4 capture compare channels of the Timer 4 is used for generating PWMs with 4 different duty cycles.

## MAIN AGORITHM:
Please refer to the report file

## Result Analysis:
* The results were satisfying as we were able to get the readings from the MPU6050 which means the I2C communication was established between the microcontroller and the MPU06050. Then the processing of these values through code also gave us the desired values which we were expecting. Hence the code written was perfect.
* The initial testing of the project gave us a lot of errors in terms of the robot not being able to balance as we would want. The various parameters were then adjusted for making the output function into give a desired behaviour.

## Conclusion:
Working on a project with STM32F407 requires lot of hard work. The
vast amount of registers that need to be configured before you can
work on the main code can be very intimidating but then once you
get a flow of the registers of STM32 the configuration becomes an
easy step and an enjoyable one too.
Working on a project like this gave us lot of hands on experience and
debugging skills. The completion of the project gave us immense
satisfaction and motivation to work on more such projects in future.
## Acknowledgement:
We would like to thank or Professor Selvakumar K who guided us in
this project and pushed us to do a project like this,




