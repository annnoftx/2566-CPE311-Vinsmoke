# Drinking Water Dispenser System
This project simulates system of water vending machine by using STM32L152RB microcontroller which written by C language and complied with Keil uVision 5. 

### ***[Video Demo](https://drive.google.com/file/d/1NFzwzWs4v0S7MbVlSmbyAGirEvFAqOMT/view)  |  [Presentation](https://docs.google.com/presentation/d/1CWizwTaAZ7ISeOlb7eEn23BdznnQlNRtSNP5hpScRiU/edit#slide=id.g2b753baf4a6_1_33)  |  [Code](https://github.com/annnoftx/2566-CPE311-Vinsmoke/blob/main/Project/src/main.c)***

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
![alt text](https://cdn.discordapp.com/attachments/855283190037217280/1206691751059005460/image.png?ex=65dcee44&is=65ca7944&hm=ef591faeaba545a4f055878455cde5cc3e197d39a9ed3329d3543b4891bdd044&)

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
![alt text](https://cdn.discordapp.com/attachments/855283190037217280/1206691653167882261/image.png?ex=65dcee2d&is=65ca792d&hm=edd1013c134faf54683bef2137a28e6831ea79a17fb3b1deb4f9e28e77005384&)
## Waterfall Model
```bash




```
## Block Diagram

![alt text](https://cdn.discordapp.com/attachments/855283190037217280/1206683017620230174/image.png?ex=65dce622&is=65ca7122&hm=a6a9db97876bba3e605d0771768ed6c1372b3b4ec80d082c99e85a8158837b17&)


## Flowchart

![alt text](https://cdn.discordapp.com/attachments/855283190037217280/1206690042807062558/FlowchartDiagram_CPE-311.drawio_2.png?ex=65dcecad&is=65ca77ad&hm=e3c5969d627f6d6bcaa9515c86af47a409fa1f03082ac903a641653220998aad&)

## Conclusion

This project provides a basic framework for simutation of Drinking Water Dispenser System, with the provided code, you can simutale the working of water dispenser machine. In addition, this project can be further expanded upon to add more advantange module such as water filter cartridge or replace water pump module that could release water with potent water suction to decrese amount of wait time.