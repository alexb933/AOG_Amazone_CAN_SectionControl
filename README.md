# AOG_Amazone_CAN_SectionControl
SectionControl based on CAN Messages for AgOpen GPS and Amazone machines

THe sketches are based on the Machine_USB_v5 Sketch from AGO... the following sketches unfortunately only support USB (sorry for that). When you need UDP -> copy code into Machine_UDP sketch
3 Sketches:
 1. This sketch supports marking of sections into AGO -> use your sprayer as usual and AGO marking will be triggered by sections of your sprayer
 2. This sketch supports AutoSection control from AGO to your sprayer -> choose Autosection Mode in AGO and the sections of Amazone will be automatically changed
 3. This sketch supports both -> you need a physical switch to change between the two modes (marking and autosection control)

See attached a components list and schematic. Also an excel file which explains the CAN Message structure.
You can use only AMATRON, or with AMACLICK and also with Joystick at the same time. Be careful when you use AMACLICK in AUtosection Mode -> turn off AMACLICK or remove it.
Attention: when using the CANBUS shield and the optional physical switch -> PIN 2,9,19,17 are not available

Be careful when using CANBUS (use the right Pins and do not current them!). Use at your own risk. I tested the code but no guarantee for none complications
At the moment November 2023 the "marking mode" (Sketch 1) only supports until 7 Sections because the CAN Code for upper sections is unknown.
Please feel free to change the code or adapt to your situation.

THANKS a lot to Valentin for support :-)
