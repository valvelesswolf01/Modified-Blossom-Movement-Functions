# Modified-Blossom-Movement-Functions
This documents my changes to the open-source robot Blossom for use in a summer research project. 
Blossom is a customizable open-hardware robot used in human-robot interaction. My changes use the body of the Blossom and my own commands made using Dynamixel SDK software to create a interactable robot.

## Sources
Michael Suguitan and Guy Hoffman. 2019. Blossom: A Handcrafted Open-Source Robot. J. Hum.-Robot Interact. 8, 1, Article 2 (March 2019), 27 pages. https://doi.org/10.1145/3310356 

https://github.com/msgtn/r0b0/blob/main/docs/blsm.md

https://github.com/agmui/blossom-public/blob/master/README.md 

https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/main/ReleaseNote.md

## How To
Follow the build instructions in the r0bo repository, print all of the 3-d parts and get the Dynamixel specific hardware. You will also need three male-female wires, three male-male wires and a USB->Uart cable. 
At the same time follow the instructions for downloading the Dynamixel SDK, I am using C++ on a Windows device but there are options for other languages and systems. The instructions also suggest using Dynamixel Wizard 2.0 for debugging and can be used to manually move the robot and document the set up for that as well. 
