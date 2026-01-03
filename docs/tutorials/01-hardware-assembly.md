# Hardware Assembly Instructions

This tutorial provides step-by-step instructions for assembling the KLASK Hardware components. Follow these steps carefully to ensure proper assembly and functionality.

## Cutting the V-Slot Beams

### Lateral frame beams

Two **360 mm** v-slot beams

![42.png](res/imgs/42.png)

### Back frame beam

One **550 mm** v-slot beam

![44.png](res/imgs/44.png)

### Bridge main beam

One **462.7 mm** v-slot beam

![43.png](res/imgs/43.png)

### Camera holder

The lengths of the two camera holder beams can be calculated as follows:

Let $L^{Cam}_1$ be the axial length that is occupied on the horizontal beam by the camera assembly, say for the purpose of a mounting plate, space for screws and nuts, etc.

If the camera is to be placed exactly above the center of the board, then the length of the horizontal beam should be $360+L^{Cam}_1/2\ \text{[mm]}$.

We rely on a $50\text{ mm}\times 50\text{ mm}$ square base plate. So the beam length should be $360+25=385\text{ mm}$.

![46a.png](res/imgs/46a.png)

Let $L^{Cam}_2$ be the distance of the camera from the board. That is, the distance between the playing plane of the Klask board and the bottom plane of the camera assembly. If this distance is defined from the objective of the camera, then the additional offset between the camera’s objective and its bottom plane needs to be taken into account.

The length of the vertical beam should be $100.63+L^{Cam}_2\ \text{[mm]}$.

We suggest $L_2^{Cam}=365\text{ mm}$ yielding a beam length of $465.63\text{ mm}$. This dimension can benefit from a margin, so we increase it to $480\text{ mm}.$

![45a.png](res/imgs/45a.png)

### Controller

With the remaining of the cut beams, provide one 26.5 mm and one 31.5 mm.

### Cutting recommendation

To spare raw material, here is how the beams should be cut out of 1 m parts in the most optimal way:

550+360+26.5+31.5 = 968 mm
360+462.7 = 822.7 mm

## Full assembly

The image below depicts the and serves as a reference for the final assembly

![33.png](res/imgs/33.png)

The gantry decomposes into four main parts.

### **The main frame**

![37.png](res/imgs/37.png)

### **The bridge**

![34.png](res/imgs/34.png)

### **The controller**

![35.png](res/imgs/35.png)

### **The camera holder**

![36.png](res/imgs/36.png)

Below are the assembly instructions for each stage.

We recommend following the tutorial in the same order as it is shown here.

## The main frame

Build **(4x)** side connectors:

![39.png](res/imgs/39.png)

| Idler pulley plate | 4 |
| --- | --- |
| Rubber foot | 4 |
| Aluminum spacer 6mm | 4 |
| L bracket | 4 |
| M5x12 | 4 |
| M5x25 | 4 |
| M5 Lock nut | 8 |

![40a.png](res/imgs/40a.png)

## Belts

To ensure that the belts conserve a constant tension for any position of the controller, the inner belt segments must be kept parallel and perpendicular. That is:

A set of parallel segments L11, L24, L21, L14 perpendicular to another set of parallel segments L12, L22, L23, L13.

The gantry is designed to be isostatic and permit calibration of each alignment.

The following procedure is recommended:

1. Permanently attach the belts to the controller on the side holding the peg.

    picture

2. Repeat for each side: wind the belt around the double idler. Adjust the lateral position of the cross plate such that the belt is parallel to the beam below. Tighten the cross plate.

    picture

3. Repeat for each side: straighten the belt all the way to the timing pulley. Adjust the lateral position of the motor holder such that the belt is parallel to the beam below. Tighten the motor holder.

    picture
4. Cross both belts from the timing pulley to the idler on the back beam of the frame. Repeat for each side: from there, reach the idler on the opposite side of the cross plates. Adjust the lateral position of the idler on the frame such that the belt is parallel to the beam below. Tighten the idler.

    picture

5. Pull the belts to the controller and apply tension. Make sure to have similar tension on both belts. Fasten the belts with the brass rings.

    picture
