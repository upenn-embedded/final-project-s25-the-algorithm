[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/WXeqVgks)
# final-project-skeleton

* Team Number: 02
* Team Name: The Algorithm
* Team Members: Derek He, Ben Saxon, Eric Li
* GitHub Repository URL: https://github.com/upenn-embedded/final-project-s25-the-algorithm
* GitHub Pages Website URL: [for final submission]

## Final Project Proposal

### 1. Abstract

*In a few sentences, describe your final project.*

We are creating a robotic screwdriver. We use a delta robot manipulator as our robotic arm, providing precise millimeter-level 3-dimensional control of the screwdriver attached to the end effector of our robotic arm. Equipped with computer vision, the robotic screwdriver contains a camera which detects the desired screws to screw in, and maps the coordinates of the screws to the corresponding motor control coordinates for the delta robot manipulator, which - in turn - will screw or unscrew the screws and assemble or disassemble our object.

### 2. Motivation

*What is the problem that you are trying to solve? Why is this project interesting? What is the intended purpose?*

With our project, we are trying to solve the problem of manual assembly for manufacturing products which include screws. This project is interesting because of its immense potential: it can be applied to almost any manufacturing process and has the potential to enhance such processes. Its intended purpose, to automate the assembly process for screws, enables us to resolve a major bottleneck in the manufacturing process and in turn increase throughput.

### 3. System Block Diagram

*Show your high level design, as done in WS1 and WS2. What are the critical components in your system? How do they communicate (I2C?, interrupts, ADC, etc.)? What power regulation do you need?*

![image](https://github.com/user-attachments/assets/eb4abf02-5a3a-4985-b01c-9b5c04f4be33)


### 4. Design Sketches

*What will your project look like? Do you have any critical design features? Will you need any special manufacturing techniques to achieve your vision, like power tools, laser cutting, or 3D printing?*

Our project will be a delta manipulator that uses camera input, a raspberry pi, and the AVR board to perform computer vision, path planning, and inverse kinematics for real-time assembly and dissasembly of parts. We will use a waterjet at Pennovation to manufacture a quarter-inch thick top plate. In addition, we will use a saw (either hand saw or angle grinder, TBD) to cut the 2020 extrusions down to size. We will use 3D printing for the end effector. 

<img width="167" alt="image" src="https://github.com/user-attachments/assets/246c85de-74aa-4393-b4c2-3160263a6c03" />
Full Assembly


<img width="158" alt="image" src="https://github.com/user-attachments/assets/de31c72b-3bb7-41ac-a999-a4601d8e977a" />
Delta Manipulator Body


<img width="168" alt="image" src="https://github.com/user-attachments/assets/5309950d-9d6b-414a-bd1a-bf023f9f7a14" />
End Effector 


### 5. Software Requirements Specification (SRS)

*Formulate key software requirements here. Think deeply on the design: What must your device do? How will you measure this during validation testing? Create 4 to 8 critical system requirements.*

*These must be testable! See the Final Project Manual Appendix for details. Refer to the table below; replace these examples with your own.*

**5.1 Definitions, Abbreviations**

Be it henceforth known, under the jurisdiction of the City of Philadelphia, Commonwealth of Pennsylvania, and United States of America, that the "ATmega328pb" will be referred to as the "MCU." The "delta robot manipulator" is the "robotic arm." The "end effector" is the part of the robotic arm which makes contact with objects: in our project, the end effector is the screwdriver attached to a brushed DC motor. The "screw drive" is the recession pattern contained in the screw head.

**5.2 Functionality**
New software requirements using only the parts we currently have. 

| ID     | Description                                                                                                                        |
| ------ | ---------------------------------------------------------------------------------------------------------------------------------- |
| SRS-01| Motor + Encoder Closed Loop: The MCU will read pulses from the magnetic encoder attached to the end effector motor to calculate the actual RPM once every 0.01 seconds. The desired RPM will be compared to the actual RPM, and the MCU will adjust the PWM duty cycle driving the brushed DC motor accordingly. If the RPM error exceeds 10% for more than 1 second, the system will report that the screwdriver is jammed, meaning that it has finished screwing the screw in and it can stop spinning.   |
| SRS-02  | Accelerometer-Based Orientation Detection: The MCU will acquire 3-axis accelerometer data from the LSM6DS3 sensor over I2C at 100 kHz every 100 ms. It will compute pitch and roll angles using trigonometric functions and apply moving average filtering over a 50-sample window to smooth the output. If either angle exceeds ±15°, the MCU will trigger an LED alert. |
| SRS-03 | Stepper motors: The MCU will control the speed of each stepper motor by adjusting the frequency of the digital pulse signals sent to the step input of the stepper motor driver. The desired rotational speed will be mapped to a corresponding step pulse frequency, with higher frequencies resulting in faster motor rotation. |
| SRS-04 | Joystick for human control: There will be a joystick which will be read by the MCU via ADC. The ADC value is a 10-bit value, and when the value is greater than 800, the stepper motors rotate clockwise one time via a pulse. When the value is greater than 300, the motor rotates counterclockwise. When the value is in between, the stepper motors are motionless. |



Original Software Requirements assuming we had all the parts. 
| ID     | Description                                                                                                                        |
| ------ | ---------------------------------------------------------------------------------------------------------------------------------- |
| SRS-01 | Camera and computer vision: Before every screw assembly operation, the camera will capture an image and report the (x, y) coordinates of all the screws which need to be assembled. Potentially, the z-coordinate can also be calculated from the image based on a known calibration of object size at a known distance, but this will be determined during testing. The angle of the screw drive (the "+") will be detected by the camera as well using OpenCV. The coordinate and angle information will be sent via I2C at 100 kHz from the Raspberry Pi connected to the camera to the MCU, and the MCU will rotate the screwdriver to an angle which matches the screw drive. |
| SRS-02 | Screwdriver engagement with screw identification: The MCU will calculate the (x, y, z) position of the end effector screwdriver once every 0.01 seconds. If no change is detected in the (x, y, z) position of the screwdriver for more than 0.20 seconds during a screw assembly operation, the MCU will realize that the screwdriver has made sufficient contact with the screw. |
| SRS-03 | Screwdriver completed screwing detection: While the screwdriver is turning the screw, the MCU will poll the motor current every 0.01 seconds and stop when the brushed DC motor attached to the screwdriver has a current which exceeds the threshold for a jam for more than 20 consecutive polls. At this point, we will know that the screw has been successfully assembled: the brushed DC motor will be turned off, and the robotic arm will assemble the next screw. |
| SRS-04 | Robot arm controls: A closed-loop feedback algorithm will be used for state estimation of each of the three stepper motors controlling the robot arm, as well as for the screwdriver position.  |
| SRS-05 | Z-axis control during screwing: A closed-loop feedback algorithm, based on data from motor encoder, will be used to track the angular displacement of the screwdriver. This information will be used to calculate the z-axis displacement of the screwdriver needed to make sufficient contact with the screw. |
| SRS-06 | Handle user input: The MCU will respond to various pin change interrupts from buttons (a START/RESUME button, PAUSE button, and emergency STOP button) |

### 6. Hardware Requirements Specification (HRS)

*Formulate key hardware requirements here. Think deeply on the design: What must your device do? How will you measure this during validation testing? Create 4 to 8 critical system requirements.*

*These must be testable! See the Final Project Manual Appendix for details. Refer to the table below; replace these examples with your own.*

**6.1 Definitions, Abbreviations**

No additional definitions are needed. 

**6.2 Functionality**

New Hardware Requirements using just the parts we have.  

| ID     | Description                                                                                                                        |
| ------ | ---------------------------------------------------------------------------------------------------------------------------------- |
| HRS-01 | Motor + Encoder Closed LoopL The brushed DC motor encoder must be capable of detecting RPM drops below 90% of target speed sustained over 1 second to flag completion of the screwing operation.  |
| HRS-02 | Accelerometer Sensor: The accelerometer must support I2C communication at 100 kHz and provide 16-bit resolution X, Y, and Z acceleration data at a minimum output rate of 10 Hz. It must maintain accuracy sufficient to detect orientation changes within ±1° and operate reliably within the MCU’s logic voltage levels. |
| HRS-04 | Buttons for human control: Two buttons must be connected to digital I/O pins on the ATmega328PB that support Pin Change Interrupts (PCINT). The electrical design must ensure clean rising or falling edge transitions suitable for interrupt triggering, with minimal signal bounce. |
| HRS-05 | Power Management: Power supply must consistently provide regulated voltage and currency capacity. A 24V and 15A Voltage Regulator is sufficient for the system. |

Original Hardware Requirements assuming we had all the parts. 

| ID     | Description                                                                                                                        |
| ------ | ---------------------------------------------------------------------------------------------------------------------------------- |
| HRS-01 | Camera and computer vision: Camera resolution must be at least 480p with 30 fps to perform precise object detection, segmentation, and evaluate coordinate positions and distances from the camera. We expect a position detection with error of 1 mm maximum.  |
| HRS-02 | Screwdriver engagement with screw identification: The resolution for the delta manipulator's stepper motor encoders must be within 0.5 mm displacement such that it can detect z-axis displacements to know if the screwdriver is stuck unable to move downwards further, implying it is engaged with the screw. |
| HRS-03 | Screwdriver completed screwing detection: The ADC that reads the DC motor encoder data must be 10-bit to have sufficient precision to detect whether the current spike indicates a jam. This resolution can be adjusted based on our tests. |
| HRS-04 | Robot arm controls: Joints must maintain positional accuracy within 0.5 mm after repeated cycles of testing. We will evaluate the repeatability of the accuracy of the position across many cycles. |
| HRS-05 | Z-axis control during screwing: The resolution for the brush motor's motor encoders must be within 1% to accurately track the angular displacement of the screwdriver. This will allow us to calculate the z-axis displacement of the screwdriver. |
| HRS-06 | Handle user input: Buttons must reliably register input with debounce latency of 20 ms. |
| HRS-07 | Communication: Communication lines must support reliable data transmission at 100 kHz without data loss, latency, or synchronization errors. I2C communication will be used across the Raspberry Pi and the ATmega328PB. |
| HRS-08 | Power Management: Power supply must consistently provide regulated voltage and currency capacity. A 24V and 15A Voltage Regulator is sufficent for the system. |

### 7. Bill of Materials (BOM)

*What major components do you need and why? Try to be as specific as possible. Your Hardware & Software Requirements Specifications should inform your component choices.*

Here is our BOM: https://docs.google.com/spreadsheets/d/1XaGem2oc90u5TyTIHajpD4cKOrVFxEH9lh6wl16X_gg/edit?usp=sharing.
We need NEMA23 motors, motor controllers, a power supply, flange couplers, NEMA23 mount brackets, ball joint rod ends, rods, geared motor (for the screwdriver), a dc motor driver, current sensor, aluminum plate, and aluminum extrusion. These are critical to assemble our structure, delta manipulator, and electronics. Please refer to the CAD for how these components attach and work together.

*In addition to this written response, copy the Final Project BOM Google Sheet and fill it out with your critical components (think: processors, sensors, actuators). Include the link to your BOM in this section.*

### 8. Final Demo Goals

*How will you demonstrate your device on demo day? Will it be strapped to a person, mounted on a bicycle, require outdoor space? Think of any physical, temporal, and other constraints that could affect your planning.*
1. Place the delta manipulator on a table with multiple samples (i.e. the screw medium such as wood or metal and screw) placed in different locations
- The screw will already be partially entered in the threading for each sample 
2. Demonstrate end effector navigating to each sample, screwing the screw in, and unscrewing it, allowing the screw to fall down 

### 9. Sprint Planning

*You've got limited time to get this project done! How will you plan your sprint milestones? How will you distribute the work within your team? Review the schedule in the final project manual for exact dates.*

| Milestone  | Functionality Achieved       | Distribution of Work                               |
| ---------- | ---------------------------- | --------------------------------------------       |
| Sprint #1  | Build delta robot            | Ordering parts, physical assembly, machining (Ben) |
| Sprint #2  | IK functionality             | Basic algorithm deployment, testing                |
| MVP Demo   | IK, standalone screwdriver   | Screwdriver assembly (Derek), path planning (Eric) |
| Final Demo | Full kinematics +screwdriver | OpenCV development (Eric)                          |

**This is the end of the Project Proposal section. The remaining sections will be filled out based on the milestone schedule.**

## Sprint Review #1

### Last week's progress
Last week, our primary focus was to receive approval for our BOM and secure outside fundraising for the additional amount necessary for our project. Both of these goals were completed, and we've made progress with mechanical assembly as well as some initial testing. Our stepper motors and motor drivers were delivered the past week, and they were both tested on an Arduino Uno. We were able to control the motor using the motor drivers from the Arduino. Regarding the mechanical assembly, we successfully manufactured our top plate (holding the three stepper motors for the Delta robot topology), as well as one bicep for each of our three robotic arms. This was done in Pennovation.

Based on our testing, we observed that the motor drivers and stepper motors successfully interface with the MCU. We also observed that our manufactured top plate and biceps were manufactured correct: the dimensions match and the parts fit well with each other. We linked a [video](https://drive.google.com/file/d/16dpmQQ3RvTcFo5i6i9GO-tgf3tGaEsDI/view?usp=drive_link) demonstrating stepper motor control with our MCU and motor driver, along with images of our precision-manufactured top plate and bicep, which you can see below.

![image](https://github.com/user-attachments/assets/4263ab9f-5866-4fe5-9de7-48d5fb81ee17)

![image](https://github.com/user-attachments/assets/7ee64358-9cd2-4ea0-9587-2e692d12a526)

### Current state of project
The state of the project right now includes custom waterjetted aluminum parts and a successful test of our stepper motor and stepper motor drivers. Although we have made progress with initial testing and mechanical assembly, we are blocked on our hardware status: everything 
we have works properly, but we're still waiting on the majority of parts to arrive, so we are mostly blocked from any further progress in our current state. The tasks that we've currently completed align with our end goal: correctly fabricating our mechanical aluminum parts is essential for our final product, and validating motor control is an important first step toward achieving precise, sub-millimeter level control of our end effector, which is our screw. 

### Next week's plan
Based on the current state of our project, next week's plan involves progress in the hardware, software, and mechanical domains. Although we are still blocked due to the majority of our parts having not yet arrived, we are able to make important progress in some important areas: 

1) Screw detection and screw divot angle detection in OpenCV on Raspberry Pi (Eric - 6 hours); considered done if the screw angle detection is within 3 degrees of the actual angle.
2) Precise motor control and closed-loop feedback for motor position based using the ATmega328pb (Derek and Ben - 6 hours); motor control and feedback is considered done if sub-3-millimeter control is achieved.
3) Establishing working I2C communication between Raspberry Pi and ATmega328pb (Derek and Eric - 4 hours); considered done if the Raspberry Pi is able to send custom information to the ATmega328pb and the ATmega328pb is able to correctly read it.
4) Fully assembling the mechanical components of our project, given that our parts arrive in a timely manner (Ben - 1 hour); considered done if all of our available parts are assembled together
5) OPTIONAL: Furthermore, if we make solid progress in these areas, we will devise and validate high-level control algorithm for our inverse kinematics for the delta robot arm (Ben - 3 hours).

Our end goal is to create sub-1-degree angle detection and sub-1-millimeter control for our motor, but for this sprint, we are primarily concerned with high-level functionality.

## Sprint Review #2

### Last week's progress

1) Assembled hardware - wire the motor drivers, voltage regulators, stepper motors. Mount onto laser-cut scaffolding. 
2) Configure Raspberry Pi with needed packages to allow for Open CV
3) In the process for establishing I2C connection between ATMega328PB with Raspberry Pi
4) Produce simulations of inverse kinematics for control of the arms - initial code written in Matlab for testing 

### Current state of project

There is progress made in all fronts: hardware assembled, raspberry pi development, and logic for controlling the stepper motors. 

### Next week's plan

1) Finetune the control of the arms via incorporating interpolation to smooth the travel of the arms across points. This needs to be developed with memory limitations in mind, because we need to precalculate the intermediate points between the start and end before the arm begins its journey
2) Implement the stepper motor control in bare metal C.
3) Implement the end effector - both hardware and bare metal control 

## MVP Demo

1. Show a system block diagram & explain the hardware implementation.
2. Explain your firmware implementation, including application logic and critical drivers you've written.

We wrote code in the main.c file that allows the MCU to control the three stepper motors. Currently, it sends pulses that can be tuned using different delays. The motor takes a step each time there is a rising edge so the speed of the stepper motor is adjusted by changing the delays. There is a maximum speed they can go due to the hardware limitations, which we found to be corresponding to around 125 us. 
We wrote code in endeffector.c that spins the DC motor and is controlled by a button and killswitch. While the killswitch port is high, the motor will spin. By default, it spins counterclockwise. When the user presses the button attached to PB2, the motor spins clockwise. 
We wrote code in test_encoder.c which takes in data from the motor encoder and stabilizes the RPM of the motor. The speed of the motor is a product of the voltage of the power it receives and the duty cycle of the PWM that is sent to the “ENA” (enable) pin on the L298N. The encoder reports the actual RPM and the error is calculated, and the duty cycle of the PWM is changed (via updating the OCR0A) 
We wrote code in delta_ik_solver.c that checks if the delta robot can reach a specific point in space and provides the joint angles required to do so, using geometric inverse kinematics and solving trigonometric equations per arm.

4. Demo your device.

Please find the videos uploaded in this github that showcase the code 

6. Have you achieved some or all of your Software Requirements Specification (SRS)?
   1. Show how you collected data and the outcomes.

We made code that can calculate the RPM of the DC motor using the encoder data. 
We made code that can calculate the RPM of the DC motor using the encoder data. 
We demonstrate the stepper motors being controlled by the speed of the pulses being sent. 
We demonstrate the buttons working through reading the states of the pins but we will later implement it with PCINT. 

7. Have you achieved some or all of your Hardware Requirements Specification (HRS)?
   1. Show how you collected data and the outcomes.
      
We showed the stepper motor requirements by testing the limits of how fast the signal must be and determined it was 8 kHz. We found the signal must be a minimum of 2 kHz or it wouldn’t be able to turn the motor. 
We demonstrate lack of debouncing for the buttons. 
We demonstrate that the power supply is able to sufficiently power the stepper motors. 

8. Show off the remaining elements that will make your project whole: mechanical casework, supporting graphical user interface (GUI), web portal, etc.

We still need to receive many mounting parts. They will allow us to achieve more functionality by allowing us to use the stepper motors to control the stepper motor. 
We need to implement the gyroscope. This was an idea only decided today upon receiving feedback from our account manager. 

10. What is the riskiest part remaining of your project?


The riskiest part of the project is whether our parts will arrive. If they do, then we can achieve a lot more of our original functionality. 
   1. How do you plan to de-risk this?

If this doesn’t happen, the new requirements specified above will be what we will achieve to still implement the principles learned in the class. 

11. What questions or help do you need from the teaching team?
None. Discussed with our account manager all the questions we had. 

## Final Project Report

Don't forget to make the GitHub pages public website!
If you’ve never made a GitHub pages website before, you can follow this webpage (though, substitute your final project repository for the GitHub username one in the quickstart guide):  [https://docs.github.com/en/pages/quickstart](https://docs.github.com/en/pages/quickstart)

### 1. Video

(https://youtu.be/op6y3o5i6jg)

### 2. Images

[Insert final project images here]

*Include photos of your device from a few angles. If you have a casework, show both the exterior and interior (where the good EE bits are!).*
![side](https://github.com/user-attachments/assets/5a04b60e-3e03-4706-b558-735ea4483088)

![bottom](https://github.com/user-attachments/assets/dbccd429-c614-4d7c-9594-dd2c10b60630)

![complete](https://github.com/user-attachments/assets/69034c65-44e9-4dcc-b036-7bff3b777460)


### 3. Results

*What were your results? Namely, what was the final solution/design to your problem?*

#### 3.1 Software Requirements Specification (SRS) Results

*Based on your quantified system performance, comment on how you achieved or fell short of your expected requirements.*

*Did your requirements change? If so, why? Failing to meet a requirement is acceptable; understanding the reason why is critical!*

*Validate at least two requirements, showing how you tested and your proof of work (videos, images, logic analyzer/oscilloscope captures, etc.).*

| ID     | Description                                                                                               | Validation Outcome                                                                          |
| ------ | --------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| SRS-01 | The IMU 3-axis acceleration will be measured with 16-bit depth every 100 milliseconds +/-10 milliseconds. | Confirmed, logged output from the MCU is saved to "validation" folder in GitHub repository. |

#### 3.2 Hardware Requirements Specification (HRS) Results

*Based on your quantified system performance, comment on how you achieved or fell short of your expected requirements.*

*Did your requirements change? If so, why? Failing to meet a requirement is acceptable; understanding the reason why is critical!*

*Validate at least two requirements, showing how you tested and your proof of work (videos, images, logic analyzer/oscilloscope captures, etc.).*

| ID     | Description                                                                                                                        | Validation Outcome                                                                                                      |
| ------ | ---------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| HRS-01 | A distance sensor shall be used for obstacle detection. The sensor shall detect obstacles at a maximum distance of at least 10 cm. | Confirmed, sensed obstacles up to 15cm. Video in "validation" folder, shows tape measure and logged output to terminal. |
|        |                                                                                                                                    |                                                                                                                         |

### 4. Conclusion

Reflect on your project. Some questions to address:

**What did you learn from it?**

**What went well?**

**What accomplishments are you proud of?**



**What did you learn/gain from this experience?**



**Did you have to change your approach?**

Yes, our approach changed in multiple ways. Our original project scope expanded beyond the time allotted for our project, so we decided to eliminate the computer vision functionality for our screw detection, opting instead to focus on our bare-metal C firmware programming, delta robot arm controls, end effector functionality, and I2C accelerometer accuracy and reliability. Additionally, since one of our critical components did not arrive, our delta robot arm was missing its forearms, and we would have had little to show for our demo. Thus, we decided to create makeshift forearms by tying wires and duct-taping them to metal rods, which ensured that we were still able to provide an impressive and successful demo.

**What could have been done differently?**

Improving our software design is one thing we could have done differently, something which would have significantly enhanced the testing and debug process. Rather than combining all of our code into a single main.c file, we should have first determined our overall file structure with header and source files, as well as defining the function behaviors and interfaces before writing actual code. This would have allowed us to isolate different subsystems in an easier way, reducing the time needed to debug our firmware.

**Did you encounter obstacles that you didn’t anticipate?**

The main unanticipated obstacle during our project development process was that almost all of our parts arrived late, and some of them simply did not arrive. As a result, our team was unable to conduct full-system integration tests to verify the functionality of our screwdriver robot. However, we were still able to execute upon much of our original vision by designing a creative substitute for our missing parts.

**What could be a next step for this project?**

Since our ultimate vision for our project is to create a fully-autonomous screwdriver robot, some exciting future steps exist for this project. Creating a screw detection algorithm, detecting the angle of the screw head and the three-dimensional coordinates of the top of the screw head would be the first step. Afterward, once we receive all of our parts, including the ball joints, we would have a full delta robot, so another step would be to validate sub-millimeter precision for controlling the delta robot arm. The third step would be to perfect the screwdriving capabilities of the end effector screwdriver, ensuring that it successfully screws in parts with high reliability. The final step would be to integrate all of the first three steps, designing a policy which determines the shortest path between a set of screws to maximize the speed of manufacturing.

## References

Fill in your references here as you work on your final project. Describe any libraries used here.
