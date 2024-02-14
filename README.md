# Drinking Water Dispenser System
This project simulates system of water vending machine by using STM32L152RB microcontroller which written by C language and complied with Keil uVision 5. 

### ***[Video Demo Case1](https://drive.google.com/file/d/1iu6bIr4y-yecUIYdDHN-dAcqR0RujI5v/view) | [Video Demo Case2](https://drive.google.com/file/d/1NFzwzWs4v0S7MbVlSmbyAGirEvFAqOMT/view)  |  [Presentation](https://docs.google.com/presentation/d/1CWizwTaAZ7ISeOlb7eEn23BdznnQlNRtSNP5hpScRiU/edit#slide=id.g2b753baf4a6_1_33)  |  [Code](https://github.com/annnoftx/2566-CPE311-Vinsmoke/blob/main/Project/src/main.c)***

## Let's get start
### Hardware
- STM32L152RB microcontroller
- LM393 Infrared Speed Sensor Module
- HC-SR04 Ultrasonic Sensor
- Mini water pump DC 3-6V 3W 
- 2 Channel 5V Active Low Relay Module
- Speaker Module
- LED & Button

### Software
- [Keil uVision 5](https://keil-vision.software.informer.com/5.0/ "Dowload Keil")

## Pin Allocation
![alt text](https://github.com/annnoftx/2566-CPE311-Vinsmoke/blob/main/pin_allocation.png)
![alt text](https://github.com/annnoftx/2566-CPE311-Vinsmoke/blob/main/pinallocation.png)

## Usage
1. Open the project in Keil uVision 5 and build the code.
2. Flash the code to the STM32L152RB microcontroller.
3. Checking water level in supply, which represents status of machine.
4. Place a bottle and enter some coins.
5. Press the buttom in the user interface to activate or stop water pump and music.

## Features
- The water dispenser can dispense 1.2 liter of water for 1 baht (accepts coins only).
- There is a coin counting system that counts the coins inserted into the system and can distinguishv types of coins.
- There is a system that displays percentage of remaining water and amount of available money in the system.
- There is a button for dispensing and stopping water.
- There is a system that dispenses water volume according to the amount of money.
Music plays while water is being dispensed.
- There is a green light represents that the system is ready to use (water level in the tank exceeds 30%).
- There is a red light represents that the system is not ready to use (water level in the tank is below 30%).

## Resources
- [STM32L152RB Datasheet](https://www.st.com/resource/en/datasheet/stm32l152rb.pdf)
- [Reference manual](https://www.st.com/resource/en/reference_manual/cd00240193-stm32l100xx-stm32l151xx-stm32l152xx-and-stm32l162xx-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf)
- [LM393 Speed Sensor](https://5.imimg.com/data5/VQ/DC/MY-1833510/lm393-motor-speed-measuring-sensor-module-for-arduino.pdf)

## Gantt chart
![alt text](https://github.com/annnoftx/2566-CPE311-Vinsmoke/blob/main/GanttChart.png)

## Waterfall Model
![alt text](https://github.com/annnoftx/2566-CPE311-Vinsmoke/blob/main/waterfall.png)

## Block Diagram
![alt text](https://github.com/annnoftx/2566-CPE311-Vinsmoke/blob/main/blockdiagram.png)


## Flowchart
![alt text](https://github.com/annnoftx/2566-CPE311-Vinsmoke/blob/main/Flowchart.png)

## Conclusion

This project provides a basic framework for simutation of Drinking Water Dispenser System, with the provided code, you can simutale the working of water dispenser machine. In addition, this project can be further expanded upon to add more advantange module such as water filter cartridge or replace water pump module that could release water with potent water suction to decrese amount of wait time.
