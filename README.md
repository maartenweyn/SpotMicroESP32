# SpotMicroESP32
My take on a SpotMicro with an optimized design for supportfree 3D-printing utilizing an ESP32-DevKitC. This is still a Work-in-Progress!

You can find the 3D-printing parts on Thingiverse as well: https://www.thingiverse.com/thing:4559827/files

Join the Slack-Channel for this remix - discuss any topics regarding this design: spotmicroai.slack.com (#spotmicro-esp32)

## Note of thanks
I want to thank Deok-yeon Kim (KDY0523) for his beatiful and well thought out design of the SpotMicro, without whom my derivate would not have been possible. You can find his original Design here on Thingiverse: https://www.thingiverse.com/thing:3445283
Him sharing the files with an open licence, made it possible to let it grow a community around it. This is why i want to reference to them as well - the SpotMicroAI Community: https://spotmicroai.readthedocs.io/en/latest/

## Preface
This is a work-in-progress, with rather loose milestones. For 2020 there were only two goals
- to redesign the original parts, so that they could be printed without any support
- to make a circuitry for all sensors and parts, that possibly would fit and be considere necessary

Both of these will be coming soon, as both goals have progress farther than 90% each. This repository is a representation of my hobby and as such, it will have its own pace. This being said, i am still happy about any questions, collaborations, suggestions, idead, forks and what else comes to your mind. I want you to participate, because i think there are a lot of things, that you could dig into.

## Coming soon...
- ~~STLs and FreeCAD-Files~~
- Mountingplate for the ciruitry - FreeCAD and STL (in progress)
- [Assembly Guide](https://github.com/michaelkubina/SpotMicroESP32/blob/master/assembly/) (in progress)
- more Template FreeCAD-Files for your own modifications
- KiCAD or Fritzing circuit diagrams
- Photos
- other things i forgot

## Sections

* [3D-printed Partlist](https://github.com/michaelkubina/SpotMicroESP32/blob/master/parts/SpotMicroESP32_parts_v1_0_0/)
* [Bill of Materials](https://github.com/michaelkubina/SpotMicroESP32/#bill-of-material)
* [Assembly Guide](https://github.com/michaelkubina/SpotMicroESP32/blob/master/assembly/)

## Bill of Material
The following Section will describe in detail the different parts needed for your own SpotMicroESP32 build, with at least some rough estimate of the price.

### 3D-Printing
To build your own SpotMicroESP32 you will need to print a lot parts. For an overview of the 3D printed parts needed, please visit the [3D-printed Partlist](https://github.com/michaelkubina/SpotMicroESP32/blob/master/parts/SpotMicroESP32_parts_v1_0_0/). It might be necessary to remix some of the templates to suit your needs, when chosing other electronic components than those listet here.

### Electronics
The SpotMicroESP32 ist still a Work-in-Progress and as such the BOM might change as well. There are still some parts missing and some others are not tested yet - those status will be marked as such. I will give a rough price estimate, which may or may not work for you and might be out of date at some point.

| part or module | number | short description | status | estimated price | 
|----------------|--------|-------------------|--------|:-------------------------|
| ESP32-DevKitC  							|  1x | the core MCU for the build with WIFI + BLE capabilities												| tested			| 7�		|
| MG996R Servo 								| 12x | 10Kg servos with metal gears and ball bearings 														| tested			| 5� each 	|
| FSH6S Servohorn 							| 12x | servohorn used for this build (should be already shipped with your servos) 							| tested 			| n.n. 		|
| Rubber Dampeners for your Servos			| 48x | servo rubber dampeners, so you could use M3 screws (should be already shipped with your servos) 	| tested 			| n.n. 		|
| Servocable Extension						|  4x | extending the servocables of the lower legs about 10cm to 15cm 										| tested			| 10�		|
| 625ZZ Ball Bearing 						|  8x | miniature ball bearing without a flange 															| tested 			| 1� each 	|
| 5mm ~3V LEDs (White)						|  6x | LEDs used as your camera-lights																		| tested			| 1�		|
| 5mm ~3V RGB-LEDs							|  2x | RGB-LEDs used as an underglow, might signal status or mood											| tested			| 1�		|
| HC-SR04 Ultrasonic Sensor					|  2x | ultrasonic sensor module for distance measuring														| tested			| 3� each	|
| GY-521 Gyroscope and Accelerometer		|  2x | a module to measure accelaration and spatial orientation, which can be extented with magnetometers  | tested			| 3� each	|
| PCA9685 16Channel 12Bit PWM Board			|  1x | PWM driver board used for your servos and LEDs, which can power your parts from an external source	| tested			| 5�		|
| OV7670 VGA-Camera Module w/o FIFO			|  1x | VGA-Camera without framebuffer IC, used with lower resolution due to memory restrictions			| tested			| 3�		|
| HW-482 5V 10A Relais						|  1x | relais module to cut the power to your servos completely											| tested			| 3�		|
| 1,77" TFT with ST7735 w/o SD				|  1x | small TFT screen with ST7735 IC for status informations												| tested			| 7�		|
| 19mm Push Button with LED					|  1x | illuminated pushbutton as external interrupt and single button input								| tested			| 10�		|
| XL4016 DC-DC 12A Stepdown Converter  		|  1x | step-down converter to lower your LiPo 2S voltage to 6V ***(better use an UBEC instead???)***		|*partially tested* | 8�		|
| 5200mAh - 6200mAh LiPo 30C+ 2S Tamiya/XH	|  1x | beefy LiPo as your main power source ***(this is actually your individual choice)***				|**not tested** 	| 40�		|

### Miscellaneous

| part or module | number | short description | status | estimated price | 
|----------------|--------|-------------------|--------|:-------------------------|
| M2x8 cylinderhead screws + M2 nuts		| 84x each | screws + nuts to mount your servohorns ***(you could glue the servohorns in place instead and save yourself 72x M2x8 screws and nuts, but i have not tested it)***, also used to mount the 1,77" TFT and the pins for the ball bearings 	| tested | 8�  |
| M3x8 cylinderhead screws + M3 nuts		| 80x each | screws + nuts for the whole assembly ***(i hope i have not forgotten some, better buy in bulk as these are usefull for other projects as well)***																							| tested | 8�  |
| M3x20 cylinderhead screws + M3 nuts		| 64x each | screws + nuts to mount your servos and assemble the upper legs																																												| tested | 12� |
| lots of cables and connectors and stuff	|          | it's up to you which cables or wires and connectors you want to use, or if you would like to solder everything into place, and if you use heat shrinks, fabric hoses 																		| up to you | 20� |
| PLA + TPU (***?***)						|          | you will need about 1Kg of PLA + some grams of TPU (eg. my Black/White design: 500g for the covers + upper legs, 500g for the chassis + lower legs + shoulder joints, some small grams for the grey sensorplate in the head) + some grams of TPU for the foottips | tested | 30�+ |