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
![image](https://github.com/user-attachments/assets/78783ea3-6ce4-4814-bfc8-8f0cb87497eb)

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

| ID     | Description                                                                                                                                                                                                              |
| ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
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

| ID     | Description                                                                                                                        |
| ------ | ---------------------------------------------------------------------------------------------------------------------------------- |
| HRS-01 | Camera and computer vision: Camera resolution must be at least 480p with 30 fps to perform precise object detection, segmentation, and evaluate coordinate positions and distances from the camera. We expect a position detection with error of 1 mm maximum.  |
| HRS-02 | Screwdriver engagement with screw identification: The resolution for the delta manipulator's stepper motor encoders must be within 0.5 mm displacement such that it can detect z-axis displacements to know if the screwdriver is stuck unable to move downwards further, implying it is engaged with the screw.  |
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
Last week, our primary focus was to receive approval for our BOM and secure outside fundraising for the additional amount necessary for our project. Both of these goals were completed, and we've made progress with mechanical assembly as well as some initial testing. Our stepper motors and motor drivers were delivered the past week, and they were both tested on an Arduino Uno. We were able to control the motor using the motor drivers from the Arduino. Regarding the mechanical assembly, we successfully manufactured our top plate (holding the three stepper motors for the Delta robot topology), as well as one bicep for each of our three robotic arms. This was done in Pennovation. We linked a video demonstrating stepper motor control with our MCU and motor driver, along with our precision-manufactured bicep and top plate, below.

https://drive.google.com/file/d/16dpmQQ3RvTcFo5i6i9GO-tgf3tGaEsDI/view?usp=drive_link

![image](https://github.com/user-attachments/assets/4263ab9f-5866-4fe5-9de7-48d5fb81ee17)

![image](https://github.com/user-attachments/assets/7ee64358-9cd2-4ea0-9587-2e692d12a526)



### Current state of project
Although we have made progress with initial testing and mechanical assembly, we are still waiting on the majority of parts to arrive, so we are mostly blocked from any further progress in our current state. However, we are still able to make progress, as outlined in the "Next week's plan" section.

### Next week's plan
Based on the current state of our project, next week's plan involves progress in the hardware, software, and mechanical domains. Although we are still blocked due to the majority of our parts having not yet arrived, we are able to make important progress in some important areas: screw detection and screw divot angle detection in OpenCV, precise motor control and closed-loop feedback for motor position based using the ATmega328pb, and fully assembling the mechanical components of our project (given that our parts arrive in a timely manner). Furthermore, if we make solid progress in these areas, we will devise a high-level policy for our inverse kinematics for the delta robot arm.

## Sprint Review #2

### Last week's progress

### Current state of project

### Next week's plan

## MVP Demo

1. Show a system block diagram & explain the hardware implementation.
2. Explain your firmware implementation, including application logic and critical drivers you've written.
3. Demo your device.
4. Have you achieved some or all of your Software Requirements Specification (SRS)?

   1. Show how you collected data and the outcomes.
5. Have you achieved some or all of your Hardware Requirements Specification (HRS)?

   1. Show how you collected data and the outcomes.
6. Show off the remaining elements that will make your project whole: mechanical casework, supporting graphical user interface (GUI), web portal, etc.
7. What is the riskiest part remaining of your project?

   1. How do you plan to de-risk this?
8. What questions or help do you need from the teaching team?

## Final Project Report

Don't forget to make the GitHub pages public website!
If you’ve never made a GitHub pages website before, you can follow this webpage (though, substitute your final project repository for the GitHub username one in the quickstart guide):  [https://docs.github.com/en/pages/quickstart](https://docs.github.com/en/pages/quickstart)

### 1. Video

[Insert final project video here]

* The video must demonstrate your key functionality.
* The video must be 5 minutes or less.
* Ensure your video link is accessible to the teaching team. Unlisted YouTube videos or Google Drive uploads with SEAS account access work well.
* Points will be removed if the audio quality is poor - say, if you filmed your video in a noisy electrical engineering lab.

### 2. Images

[Insert final project images here]

*Include photos of your device from a few angles. If you have a casework, show both the exterior and interior (where the good EE bits are!).*

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

* What did you learn from it?
* What went well?
* What accomplishments are you proud of?
* What did you learn/gain from this experience?
* Did you have to change your approach?
* What could have been done differently?
* Did you encounter obstacles that you didn’t anticipate?
* What could be a next step for this project?

## References

Fill in your references here as you work on your final project. Describe any libraries used here.
