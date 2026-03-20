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

The explosion view below shows how the controller is assembled. The Mini V Gantry Kit comes pre-assembled and the longer of the two V-Slot beams is used on the side where the magnet is mounted.

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

The following explosion view shows how the bridge is assembled.

![bridge explosion](../res/imgs/tutorials/hw-assembly/bridge_iso_exp.png){width="800"}

First assemble one side, then slide the controller onto the bridge beam, and finally assemble the other side.

To later have an aligned idler wheel with the motor wheel, mount the cross plate with a distance of 11 mm from the end of the bridge beam as shown in the image below.

![bridge beam distance](../res/imgs/tutorials/hw-assembly/bridge_beam_dist.png){width="800"}

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
| 7    | Countersunk Screw | M3x14                            | 12       |                                                                                               |
| 8    | Hex Nut           | M2                               | 8        |                                                                                               |
| 9    | Hex Nut           | M3                               | 12       |                                                                                               |
| 10   | Tee Nut           | M5                               | 8        | [Link](https://us.openbuilds.com/tee-nuts-m5-pack/)                                           |

It is rather simple to assemble as you can see from the explosion view below.

![frame support explosion](../res/imgs/tutorials/hw-assembly/frame_support_iso_exp.png){width="800"}

The hight of the limit switch has to be adjusted such that the actuator on the switch is pressed when the centering bridge of the board frame (which is the counter part of the cone shape) rests in it.

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

The following explosion view shows how to assemble the casing.

![limit switch mount explosion](../res/imgs/tutorials/hw-assembly/limit_switch_l_iso_exp.png){width="800"}

![limit switch mount assembly](../res/imgs/tutorials/hw-assembly/limit_switch_r_iso.png){width="800"}

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

Start by assembling the main frame structure as shown im the image below.

![gantry frame main structure](../res/imgs/tutorials/hw-assembly/gantry_frame_main_iso.png){width="800"}

![gantry frame main structure explosion](../res/imgs/tutorials/hw-assembly/gantry_frame_main_iso_exp.png){width="800"}

The distance between the x-beam and the y-beam is 33.7.

![gantry frame y dist](../res/imgs/tutorials/hw-assembly/gantry_frame_y_dist.png){width="800"}

Next we build the motor mounts.

![motor mount](../res/imgs/tutorials/hw-assembly/motor_mount_iso.png){width="600"}

The explosion view below shows how to assemble the motor mounts.

![motor mount explosion](../res/imgs/tutorials/hw-assembly/motor_mount_iso_exp.png){width="400"}

Now you can fix the motor mounts to the previously assembled main frame structure. After that you can slide the bridge onto the gantry frame.

![gantry frame bridge mounting](../res/imgs/tutorials/hw-assembly/gantry_frame_bridge_mounting.png){width="800"}

Now you can finish the frame structure by adding the idler wheels and the feet.

![gantry frame front mounting](../res/imgs/tutorials/hw-assembly/gantry_frame_front_mounting.png){width="400"}

With the gantry frame structure finishes we can now add the board frame support and the limit switch mounts.

![gantry frame final mounting](../res/imgs/tutorials/hw-assembly/gantry_frame_final_mounting.png){width="800"}

### Belt Tensioning

To mount and tension the belts properly we recommend to first loosen the idler pulleys indicated in the image below.

![belt tensioning](../res/imgs/tutorials/hw-assembly/belt_tensioning_1.png){width="800"}

Now fix one end of the first belt to the controller and rout it around the pulleys as shown in the image below.

![belt routing](../res/imgs/tutorials/hw-assembly/belt_tensioning_2.png){width="800"}

Repeat the same for the second belt.

![belt routing](../res/imgs/tutorials/hw-assembly/belt_tensioning_3.png){width="800"}

Finally tension the belt by pulling the loose idler pulley away from the center and re-tighten it.

## Overall BOM

A summary of all parts needed to build the complete KLASK hardware system is provided below. Quantities are for 1 camera mount + 1 board frame + 1 gantry system (with quantities for 2 gantry systems shown in brackets).

| Pos. | Name                            | Description                      | Quantity | Link                                                                                          |
| ---- | ------------------------------- | -------------------------------- | -------- | --------------------------------------------------------------------------------------------- |
|      | V-Slot 20x20 Linear Rail        | L = 660                          | 1        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)                             |
|      | V-Slot 20x20 Linear Rail        | L = 600                          | 2        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)                             |
|      | V-Slot 20x20 Linear Rail        | L = 550                          | 1 (2)    | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)                             |
|      | V-Slot 20x20 Linear Rail        | L = 500                          | 6 (8)    | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)                             |
|      | V-Slot 20x20 Linear Rail        | L = 462.7                        | 1 (2)    | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)                             |
|      | V-Slot 20x20 Linear Rail        | L = 360                          | 4 (6)    | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)                             |
|      | V-Slot 20x20 Linear Rail        | L = 340                          | 2        | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)                             |
|      | V-Slot 20x20 Linear Rail        | L = 31.5                         | 1 (2)    | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)                             |
|      | V-Slot 20x20 Linear Rail        | L = 26.5                         | 1 (2)    | [Link](https://openbuildspartstore.com/v-slot-20x20-linear-rail/)                             |
|      | Mini V Gantry Kit               |                                  | 3 (6)    | [Link](https://openbuildspartstore.com/mini-v-gantry-kit/)                                    |
|      | Black Angle Corner Connector    |                                  | 18 (26)  | [Link](https://openbuildspartstore.com/black-angle-corner-connector/)                         |
|      | End Cap                         |                                  | 18 (26)  | [Link](https://openbuildspartstore.com/end-cap/)                                              |
|      | Inside Hidden Corner Bracket    |                                  | 2 (4)    | [Link](https://openbuildspartstore.com/inside-hidden-corner-bracket/)                         |
|      | Cross Joining Plate             |                                  | 2 (4)    | [Link](https://openbuildspartstore.com/cross-joining-plate/)                                  |
|      | Low Profile Screw               | M5x8                             | 58 (88)  | [Link](https://us.openbuilds.com/low-profile-screws-10-pack/)                                 |
|      | Low Profile Screw               | M5x25                            | 2 (4)    | [Link](https://us.openbuilds.com/low-profile-screws-10-pack/)                                 |
|      | Low Profile Screw               | M5x30                            | 4 (8)    | [Link](https://us.openbuilds.com/low-profile-screws-10-pack/)                                 |
|      | Tee Nut                         | M5                               | 50 (72)  | [Link](https://us.openbuilds.com/tee-nuts-m5-pack/)                                           |
|      | Double Tee Nut                  | M5                               | 7 (14)   | [Link](https://openbuildspartstore.com/double-tee-nut/)                                       |
|      | Self Tapping Screw              |                                  | 7 (14)   | [Link](https://us.openbuilds.com/self-tapping-screws-10-pack/)                                |
|      | Slot Washer - 15x5x2mm          |                                  | 8 (16)   | [Link](https://openbuildspartstore.com/slot-washer-15x5x2mm-10-pack/)                         |
|      | Precision Shim - 10x5x1mm       |                                  | 14 (28)  | [Link](https://openbuildspartstore.com/precision-shim-10x5x1mm-10-pack/)                      |
|      | Aluminum Spacers                | 3mm                              | 6 (12)   | [Link](https://openbuildspartstore.com/aluminum-spacers-10-pack/)                             |
|      | Aluminum Spacers                | 9mm                              | 3 (6)    | [Link](https://openbuildspartstore.com/aluminum-spacers-10-pack/)                             |
|      | Smooth Idler Pulley Kit         |                                  | 10 (20)  | [Link](https://openbuildspartstore.com/smooth-idler-pulley-kit/)                              |
|      | GT2-2M Timing Pulley - 20 Tooth |                                  | 2 (4)    | [Link](https://openbuildspartstore.com/gt2-2m-timing-pulley-20-tooth/)                        |
|      | Idler Pulley Plate              |                                  | 2 (4)    | [Link](https://openbuildspartstore.com/idler-pulley-plate/)                                   |
|      | GT2-2M Timing Belt              | 4 m = 13 ft                      | 1 (2)    | [Link](https://openbuildspartstore.com/gt2-2m-timing-belt-by-the-foot/)                       |
|      | Nylon Insert Hex Locknut        | M5                               | 2 (4)    | [Link](https://openbuildspartstore.com/nylon-insert-hex-locknut---m5-10-pack-/)               |
|      | Rubber Foot                     |                                  | 8 (12)   | [Link](https://us.openbuilds.com/rubber-feet-set-4-pack/)                                     |
|      | Limit Switch                    | Würth WS-MITV THT (463093370402) | 6 (12)   | [Link](https://www.digikey.ch/de/products/detail/w%C3%BCrth-elektronik/463093370402/10056400) |
|      | Socket Head Screw               | M2x12                            | 8 (16)   |                                                                                               |
|      | Socket Head Screw               | M4x20                            | 8 (16)   |                                                                                               |
|      | Socket Head Screw               | M5x30                            | 8 (16)   |                                                                                               |
|      | Countersunk Screw               | M2x12                            | 4 (8)    |                                                                                               |
|      | Countersunk Screw               | M3x14                            | 12 (24)  |                                                                                               |
|      | Countersunk Screw               | M5x18                            | 2 (4)    |                                                                                               |
|      | Hex Nut                         | M2                               | 12 (24)  |                                                                                               |
|      | Hex Nut                         | M3                               | 12 (24)  |                                                                                               |
|      | ODrive S1 and M8325s Motor Kit  |                                  | 2 (4)    | [Link](https://shop.odriverobotics.com/products/s1-and-m8325s-start-kit)                      |
|      | Camera                          |                                  | 1        |                                                                                               |
|      | Nanlite Compac 24B              |                                  | 2        |                                                                                               |
|      | Frame Centering Bridge          | 3D-Print                         | 4        |                                                                                               |
|      | Camera Mounting Plate           | 3D-Print                         | 1        |                                                                                               |
|      | Peg Holder                      | 3D-Print                         | 1 (2)    |                                                                                               |
|      | Ground Plate                    | 3D-Print                         | 4 (8)    |                                                                                               |
|      | Center Plate                    | 3D-Print                         | 4 (8)    |                                                                                               |
|      | Mounting Bracket                | 3D-Print                         | 4 (8)    |                                                                                               |
|      | Mount                           | 3D-Print                         | 1 (2)    |                                                                                               |
|      | Mount mirrored                  | 3D-Print                         | 1 (2)    |                                                                                               |
|      | Cover                           | 3D-Print                         | 1 (2)    |                                                                                               |
|      | Cover mirrored                  | 3D-Print                         | 1 (2)    |                                                                                               |
|      | Motor Connector                 | 3D-Print                         | 2 (4)    |                                                                                               |
|      | Motor Enclosure Bottom          | 3D-Print                         | 2 (4)    |                                                                                               |
|      | Motor Enclosure Top             | 3D-Print                         | 2 (4)    |                                                                                               |
