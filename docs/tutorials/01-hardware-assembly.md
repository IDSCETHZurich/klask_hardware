# Hardware Assembly Instructions

This tutorial provides step-by-step instructions for assembling the KLASK Hardware components. Follow these steps carefully to ensure proper assembly and functionality.

## Overview

The KLASK robotic system consists of three main assembly groups, the **board frame** fixing the playing field, the **gantry system** providing the motion mechanism for each player side, and the **camera mount** mounting the camera and the lights above the board.

![global assembly](../res/imgs/tutorials/hw-assembly/global_assembly_iso.png){width="800"}

The following sections provide detailed assembly instructions for each of these groups. The CAD models shown here and the files for 3D printing can be found in this repo under `hardware/cad/`. A complete Bill of Materials (BOM) for all parts needed for the assembly is provided at the end of this tutorial.

## Board Frame

![board frame](../res/imgs/tutorials/hw-assembly/board_frame_w_board_iso.png){width="800"}

### Bill of Materials (BOM) for the Board Frame

The board frame consists of the following parts:

| Pos. | Name                         | Description | Quantity | Link                                                                  |
| ---- | ---------------------------- | ----------- | -------- | --------------------------------------------------------------------- |
| 1    | V-Slot 20x20 Linear Rail     | L = 500     | 2        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)     |
| 2    | V-Slot 20x20 Linear Rail     | L = 340     | 2        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)     |
| 3    | Frame Centering Bridge       | 3D-Print    | 4        |                                                                       |
| 4    | Black Angle Corner Connector |             | 4        | [Link](https://openbuildspartstore.com/black-angle-corner-connector/) |
| 5    | End Cap                      |             | 4        | [Link](https://openbuildspartstore.com/end-cap/)                      |
| 6    | Low Profile Screw            | M5x8        | 12       | [Link](https://us.openbuilds.com/low-profile-screws-10-pack/)         |
| 7    | Tee Nut                      | M5          | 12       | [Link](https://us.openbuilds.com/tee-nuts-m5-pack/)                   |

### Assembly Instructions for the Board Frame

The assembly of the board frame is straightforward. Start by cutting the V-slot beams to the specified lengths if not already done and print the frame centering bridges. Once all parts are ready you can simply assemble them as shown in the image below. Make sure that the KLASK board fits tightly into the frame. The exact Pos.ing of the frame centering bridges can be adjusted later.

![board frame assembly](../res/imgs/tutorials/hw-assembly/board_frame_iso.png){width="800"}

Thats all for the board frame assembly! Wuhoo! You've built the first part of your KLASK robot.

## Camera Mount

![camera mount](../res/imgs/tutorials/hw-assembly/camera_mount_iso.png){width="800"}

### BOM for the Camera Mount

The camera mount consists of the following parts:

| Pos. | Name                         | Description | Quantity | Link                                                                  |
| ---- | ---------------------------- | ----------- | -------- | --------------------------------------------------------------------- |
| 1    | V-Slot 20x20 Linear Rail     | L = 660     | 1        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)     |
| 2    | V-Slot 20x20 Linear Rail     | L = 600     | 2        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)     |
| 3    | V-Slot 20x20 Linear Rail     | L = 500     | 2        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)     |
| 4    | Black Angle Corner Connector |             | 6        | [Link](https://openbuildspartstore.com/black-angle-corner-connector/) |
| 5    | End Cap                      |             | 6        | [Link](https://openbuildspartstore.com/end-cap/)                      |
| 6    | Low Profile Screw            | M5x8        | 16       | [Link](https://us.openbuilds.com/low-profile-screws-10-pack/)         |
| 7    | Tee Nut                      | M5          | 16       | [Link](https://us.openbuilds.com/tee-nuts-m5-pack/)                   |
| 8    | Rubber Foot                  |             | 4        | [Link](https://us.openbuilds.com/rubber-feet-set-4-pack/)             |
| 9    | Camera                       |             | 1        |                                                                       |
| 10   | Camera Mounting Plate        | 3D-Print    | 1        |                                                                       |
| 11   | Nanlite Compac 24B           |             | 2        |                                                                       |

### Assembly Instructions for the Camera Mount

As with the board frame the camera mount assembly is quite simple. Start by cutting the V-slot beams to the specified lengths if not already done and then assemble the parts as shown in the image above.

To achieve the most robust scene lightning we recommend to set the `DIM`and `CCT` turning knobs on the Nanlite Compac 24B lights to maximum `+`.

## Gantry System

![gantry system](../res/imgs/tutorials/hw-assembly/gantry_iso.png){width="800"}

Now the most complex part of the assembly, the gantry system. This part provides the motion mechanism for the KLASK robot.

The parts needed to build **one** gantry system are listed below. We divide the BOM and the instructions into different sections for each sub-assembly.

### Controller

First we start with the controller assembly that mounts the magnet to play the game.

![controller](../res/imgs/tutorials/hw-assembly/controller_iso.png){width="800"}

The parts needed for that are listed below.

| Pos. | Name                         | Description | Quantity | Link                                                                     |
| ---- | ---------------------------- | ----------- | -------- | ------------------------------------------------------------------------ |
| 1    | Mini V Gantry Kit            |             | 1        | [Link](https://openbuildspartstore.com/mini-v-gantry-kit/)               |
| 2    | V-Slot 20x20 Linear Rail     | L = 26.5    | 1        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)        |
| 3    | V-Slot 20x20 Linear Rail     | L = 31.5    | 1        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)        |
| 4    | Black Angle Corner Connector |             | 2        | [Link](https://openbuildspartstore.com/black-angle-corner-connector/)    |
| 5    | End Cap                      |             | 2        | [Link](https://openbuildspartstore.com/end-cap/)                         |
| 6    | Low Profile Screw            | M5x8        | 2        | [Link](https://us.openbuilds.com/low-profile-screws-10-pack/)            |
| 7    | Low Profile Screw            | M5x30       | 2        | [Link](https://us.openbuilds.com/low-profile-screws-10-pack/)            |
| 8    | Self Tapping Screw           |             | 3        | [Link](https://us.openbuilds.com/self-tapping-screws-10-pack/)           |
| 9    | Tee Nut                      | M5          | 2        | [Link](https://us.openbuilds.com/tee-nuts-m5-pack/)                      |
| 10   | Slot Washer - 15x5x2mm       |             | 8        | [Link](https://openbuildspartstore.com/slot-washer-15x5x2mm-10-pack/)    |
| 11   | Precision Shim - 10x5x1mm    |             | 8        | [Link](https://openbuildspartstore.com/precision-shim-10x5x1mm-10-pack/) |
| 12   | Aluminum Spacers             | 3mm         | 6        | [Link](https://openbuildspartstore.com/aluminum-spacers-10-pack/)        |
| 13   | Peg Holder                   | 3D-Print    | 1        |                                                                          |

The explosion view below shows how the controller is assembled. The Mini V Gantry Kit comes pre-assembled. Ant the longer of the two V-Slot beams is used on the side where the magnet is mounted.

![controller explosion](../res/imgs/tutorials/hw-assembly/controller_iso_exp.png){width="800"}

![controller assembly bottom](../res/imgs/tutorials/hw-assembly/controller_iso_b.png){width="800"}

### Bridge

The bridge assembly carries the controller that moves along the Y axis.

![bridge](../res/imgs/tutorials/hw-assembly/bridge_iso.png){width="800"}

The parts needed for the bridge assembly are listed below.

| Pos. | Name                         | Description | Quantity | Link                                                                  |
| ---- | ---------------------------- | ----------- | -------- | --------------------------------------------------------------------- |
| 1    | V-Slot 20x20 Linear Rail     | L = 462.7   | 1        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)     |
| 2    | Mini V Gantry Kit            |             | 2        | [Link](https://openbuildspartstore.com/mini-v-gantry-kit/)            |
| 3    | Black Angle Corner Connector |             | 2        | [Link](https://openbuildspartstore.com/black-angle-corner-connector/) |
| 4    | Cross Joining Plate          |             | 2        | [Link](https://openbuildspartstore.com/cross-joining-plate/)          |
| 5    | Smooth Idler Pulley Kit      |             | 4        | [Link](https://openbuildspartstore.com/smooth-idler-pulley-kit/)      |
| 6    | Low Profile Screw            | M5x8        | 8        | [Link](https://us.openbuilds.com/low-profile-screws-10-pack/)         |
| 7    | Low Profile Screw            | M5x25       | 2        | [Link](https://us.openbuilds.com/low-profile-screws-10-pack/)         |
| 8    | Low Profile Screw            | M5x30       | 2        | [Link](https://us.openbuilds.com/low-profile-screws-10-pack/)         |
| 9    | Tee Nut                      | M5          | 2        | [Link](https://us.openbuilds.com/tee-nuts-m5-pack/)                   |
| 10   | Double Tee Nut               | M5          | 2        | [Link](https://openbuildspartstore.com/double-tee-nut/)               |
| 11   | Self Tapping Screw           |             | 2        | [Link](https://us.openbuilds.com/self-tapping-screws-10-pack/)        |
| 12   | End Cap                      |             | 2        | [Link](https://openbuildspartstore.com/end-cap/)                      |

### Board Frame Support

The board frame support connects the gantry system to the board frame.

![board frame support](../res/imgs/tutorials/hw-assembly/frame_support_iso.png){width="800"}

It consists of the following parts:

| Pos. | Name              | Description                      | Quantity | Link                                                                                          |
| ---- | ----------------- | -------------------------------- | -------- | --------------------------------------------------------------------------------------------- |
| 1    | Ground Plate      | 3D-Print                         | 4        |                                                                                               |
| 2    | Center Plate      | 3D-Print                         | 4        |                                                                                               |
| 3    | Mounting Bracket  | 3D-Print                         | 4        |                                                                                               |
| 4    | Limit Switch      | Würth WS-MITV THT (463093370402) | 4        | [Link](https://www.digikey.ch/de/products/detail/w%C3%BCrth-elektronik/463093370402/10056400) |
| 5    | Socket Head Screw | M2x12                            | 8        |                                                                                               |
| 6    | Socket Head Screw | M5x30                            | 8        |                                                                                               |
| 7    | Countersunk Screw | M3x25                            | 12       |                                                                                               |
| 8    | Hex Nut           | M2                               | 8        |                                                                                               |
| 9    | Hex Nut           | M3                               | 12       |                                                                                               |
| 10   | Tee Nut           | M5                               | 8        | [Link](https://us.openbuilds.com/tee-nuts-m5-pack/)                                           |

### Limit Switch Mount

The limit switch mount holds the limit switches for the axis end stops.

![limit switch mount](../res/imgs/tutorials/hw-assembly/limit_switch_l_iso.png){width="800"}

You need two of them a normal and a mirrored one. The parts needed are listed below.

| Pos. | Name              | Description                      | Quantity | Link                                                                                          |
| ---- | ----------------- | -------------------------------- | -------- | --------------------------------------------------------------------------------------------- |
| 1    | Mount             | 3D-Print                         | 1        |                                                                                               |
| 2    | Mount mirrored    | 3D-Print                         | 1        |                                                                                               |
| 3    | Cover             | 3D-Print                         | 1        |                                                                                               |
| 4    | Cover mirrored    | 3D-Print                         | 1        |                                                                                               |
| 5    | Limit Switch      | Würth WS-MITV THT (463093370402) | 2        | [Link](https://www.digikey.ch/de/products/detail/w%C3%BCrth-elektronik/463093370402/10056400) |
| 6    | Countersunk Screw | M2x12                            | 4        |                                                                                               |
| 7    | Countersunk Screw | M5x18                            | 2        |                                                                                               |
| 8    | Hex Nut           | M2                               | 4        |                                                                                               |
| 9    | Tee Nut           | M5                               | 2        | [Link](https://us.openbuilds.com/tee-nuts-m5-pack/)                                           |

### Gantry Frame

And now the part that holds everything together, the gantry frame.

![gantry frame](../res/imgs/tutorials/hw-assembly/gantry_frame_iso.png){width="800"}

The parts needed for the gantry frame are listed below.

| Pos. | Name                            | Description | Quantity | Link                                                                            |
| ---- | ------------------------------- | ----------- | -------- | ------------------------------------------------------------------------------- |
| 1    | V-Slot 20x20 Linear Rail        | L = 550     | 1        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)               |
| 2    | V-Slot 20x20 Linear Rail        | L = 360     | 2        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)               |
| 3    | Inside Hidden Corner Bracket    |             | 2        | [Link](https://openbuildspartstore.com/inside-hidden-corner-bracket/)           |
| 4    | Double Tee Nut                  | M5          | 5        | [Link](https://openbuildspartstore.com/double-tee-nut/)                         |
| 5    | Low Profile Screw               | M5x8        | 18       | [Link](https://us.openbuilds.com/low-profile-screws-10-pack/)                   |
| 6    | GT2-2M Timing Pulley - 20 Tooth |             | 2        | [Link](https://openbuildspartstore.com/gt2-2m-timing-pulley-20-tooth/)          |
| 7    | Idler Pulley Plate              |             | 2        | [Link](https://openbuildspartstore.com/idler-pulley-plate/)                     |
| 8    | Smooth Idler Pulley Kit         |             | 6        | [Link](https://openbuildspartstore.com/smooth-idler-pulley-kit/)                |
| 9    | Tee Nut                         | M5          | 8        | [Link](https://us.openbuilds.com/tee-nuts-m5-pack/)                             |
| 10   | Black Angle Corner Connector    |             | 2        | [Link](https://openbuildspartstore.com/black-angle-corner-connector/)           |
| 11   | End Cap                         |             | 4        | [Link](https://openbuildspartstore.com/end-cap/)                                |
| 12   | Precision Shim - 10x5x1mm       |             | 6        | [Link](https://openbuildspartstore.com/precision-shim-10x5x1mm-10-pack/)        |
| 13   | Aluminum Spacers                | 9mm         | 3        | [Link](https://openbuildspartstore.com/aluminum-spacers-10-pack/)               |
| 14   | GT2-2M Timing Belt              | 4 m = 13 ft | 1        | [Link](https://openbuildspartstore.com/gt2-2m-timing-belt-by-the-foot/)         |
| 15   | Nylon Insert Hex Locknut        | M5          | 2        | [Link](https://openbuildspartstore.com/nylon-insert-hex-locknut---m5-10-pack-/) |
| 16   | Self Tapping Screw              |             | 2        | [Link](https://us.openbuilds.com/self-tapping-screws-10-pack/)                  |
| 17   | Rubber Foot                     |             | 4        | [Link](https://us.openbuilds.com/rubber-feet-set-4-pack/)                       |
| 18   | Motor Connector                 | 3D-Print    | 2        |                                                                                 |
| 19   | Motor Enclosure Bottom          | 3D-Print    | 2        |                                                                                 |
| 20   | Motor Enclosure Top             | 3D-Print    | 2        |                                                                                 |
| 21   | ODrive S1 and M8325s Motor Kit  |             | 2        | [Link](https://shop.odriverobotics.com/products/s1-and-m8325s-start-kit)        |
| 22   | Socket Head Screw               | M4x20       | 8        |                                                                                 |

## Cutting the V-Slot Beams

### Lateral frame beams

Two **360 mm** v-slot beams

![42.png](../res/imgs/tutorials/hw-assembly/42.png)

### Back frame beam

One **550 mm** v-slot beam

![44.png](../res/imgs/tutorials/hw-assembly/44.png)

### Bridge main beam

One **462.7 mm** v-slot beam

![43.png](../res/imgs/tutorials/hw-assembly/43.png)


### Controller

With the remaining of the cut beams, provide one 26.5 mm and one 31.5 mm.


## Full assembly

The image below depicts the and serves as a reference for the final assembly

![33.png](../res/imgs/tutorials/hw-assembly/33.png)

The gantry decomposes into four main parts.

### **The main frame**

![37.png](../res/imgs/tutorials/hw-assembly/37.png)

### **The bridge**

![34.png](../res/imgs/tutorials/hw-assembly/34.png)

### **The controller**

![35.png](../res/imgs/tutorials/hw-assembly/35.png)

### **The camera holder**

![36.png](../res/imgs/tutorials/hw-assembly/36.png)

Below are the assembly instructions for each stage.

We recommend following the tutorial in the same order as it is shown here.

## The main frame

Start by building the motor mounts. The steppers are fixed onto the motor mount plates. The timing pulleys need to be mounted in opposing direction to fit the two belt circuits. Insert four tee nuts into the beam before tightening the motor mounts onto the back frame beam. Two will serve for the single idler pulleys, and two for the camera truss. Mount each single idler at different heights. Use angle connectors to attach the Y axis beams to the back frame beam. Finally, add the rubber feet.

![1.png](../res/imgs/tutorials/hw-assembly/1.png)

Build **(4x)** side connectors:

![39.png](../res/imgs/tutorials/hw-assembly/39.png)

| Idler pulley plate  | 4   |
| ------------------- | --- |
| Rubber foot         | 4   |
| Aluminum spacer 6mm | 4   |
| L bracket           | 4   |
| M5x12               | 4   |
| M5x25               | 4   |
| M5 Lock nut         | 8   |

![40a.png](../res/imgs/tutorials/hw-assembly/40a.png)

## Carriage

The carriage is built around a Mini-V kit. Two angle connectors facing opposing directions are mounted using angle connectors. The belt attachments consist of a stack of shims and spacers. The peg is held by the clamp that is mounted on the front (longer) carriage beam.

![2.png](../res/imgs/tutorials/hw-assembly/2.png)

## Bridge

Slide the carriage onto the bridge beam. On each end of the bridge beam, assemble a cross plate with two idler pulleys that lie on different planes. On the bottom side, attach Mini-V kits.

![3.png](../res/imgs/tutorials/hw-assembly/3.png)

## Bridge and carriage

The final assembly of the bridge with its carriage should look like this:

![4.png](../res/imgs/tutorials/hw-assembly/4.png)

## Y axis assembly

On the front of the Y axis beams, mount the three following subassemblies. Using two idler mount plates, create the lateral board spacer and the double idler mount, that both have an L-connector carrying a rubber foot. On the bottom of the beam, mount a rubber foot. On only one of the Y axis assemblies, add the dual connector.

![5.png](../res/imgs/tutorials/hw-assembly/5.png)

## Full frame

Slide two Y axis beam assemblies through the Mini-V kits on the bridge. Tightly connect with the back frame assembly using both interior and exterior angle connectors. Route both belt circuits (Section 6.4) around all pulleys and the timing gears for a fully assembled frame:

![6.png](../res/imgs/tutorials/hw-assembly/6.png)

## Relevant assembly offsets

For the main frame, the offsets provided here are meant to guide the construction of the gantry.

### Offset between the cross plate and the bridge beam

The width of the cross plates is 60 mm, so its center line is at 30 mm. The idler pulley is located on the center of the V-slot beam and has diameter 17.5 mm. The thickness of the belts is 1.5 mm, including the teeth. The shift between the idlers therefore needs to be 17.5 + 1.5 = 19 mm. This yields an offset between the cross plate and the end of the bridge beam of 30 − 19 = 11 mm.

![7.png](../res/imgs/tutorials/hw-assembly/7.png)

### Offset between the stepper motor and the back frame beam

The stepper motor has width 42.22 mm. Its shaft should be aligned with the beam axis at 20 mm. The offset needs to be −1.11 mm.

![8.png](../res/imgs/tutorials/hw-assembly/8.png)

### Offset between the back frame beam and the Y axis beam

The alignment of the stepper shaft and the idler pulley is at 20 + 10 = 30 mm. The idler pulley has radius 8.75 mm and the timing gear has radius 6.16 mm. The needed offset is 2.65 mm. Include an additional 1 mm accounting for the thickness of the belt. The total offset is 30 + 2.65 + 1 = 33.65 mm.

![9.png](../res/imgs/tutorials/hw-assembly/9.png)

### Offset of the back frame idler pulley and the frame

The idler pulley that sits on the back frame needs to be aligned with the idler pulley on the cross plate of the bridge. The offset is 9 mm to the inner of the gantry.

![10.png](../res/imgs/tutorials/hw-assembly/10.png)

## Belt tensioning procedure

The frame of the gantry provides the necessary degrees of freedom that allow the fine tuning of the belt alignments. The following procedure is recommended for installing the belts.

1. Move the double idler mounts about 5 mm inwards, towards the bridge. This will later allow for further tightening of the belts.
2. Permanently attach the belts to the attachments on the front of the carriage which carries the peg clamp.
3. Repeat for both sides: wind the belt around the double idler mount. Adjust the lateral Pos. of the cross plate such that the belt is parallel to the beam below. Tighten the cross plate on the bridge.
4. Repeat for both sides: straighten the belt all the way to the timing gear. Adjust the lateral Pos. of the motor mount such that the belt is parallel to the Y axis beam below. Tighten the motor mount.
5. Cross both belts from the timing gears to the single idler on the back beam of the frame. Repeat for both sides: from there, reach the pulley on the opposite side of the cross plates. Adjust the lateral Pos. of the single idler on the back beam such that the belt is parallel to the Y axis beam below. Tighten the single idler.
6. Pull the belts towards the carriage and apply tension. Make sure to have similar tension on both belts. Fasten the belts with the brass rings.
7. If more tension is required, loosen the mount plate of the double idler, pull away from the bridge, and tighten again.

Now the gantry is fully assembled and ready for integration with the electronics and software components as described in the subsequent tutorials.


## Overall BOM

A summary of all parts needed to build the KLASK hardware gantry system is provided below.

| Pos. | Name | Description | Quantity | Link |
| ---- | ---- | ----------- | -------- | ---- |