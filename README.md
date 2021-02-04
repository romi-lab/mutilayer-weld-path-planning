# Mutilayer weld path planning

## Overview
We introduce a method for V-shape groove mutilayer path planning. This method plans the welding layers, sequence, as well as all welding points (with pose in 2d) for V-shape groove. 
In the algorithm, the welding bead geometry has been approximated into parallelograms or trapezoids for simplification. 
And the manner for welding path is planned in a side-to-center, left-to-right, bottom-to-top manner. 
It allows such calculation on V-groove with various thickness, assembly clearance, and groove angle.

<img src="https://github.com/romi-lab/mutilayer-weld-path-planning/blob/main/mutilayer.gif" alt="">

## Procedures
The code mainly follows 5 steps:
1. Groove definition
2. Layer cutting
3. Segmentation
4. Geometry points identification
5. Weld path planning

## Results
<img src="https://github.com/romi-lab/mutilayer-weld-path-planning/blob/main/mutilayer.png" alt="">

## Note
For detail, please check the [document](https://github.com/romi-lab/mutilayer-weld-path-planning/blob/main/Mutilayer%20Weld%20Path%20Planning.pdf) and [code](https://github.com/romi-lab/mutilayer-weld-path-planning/blob/main/mutilayer%20weld%20path%20planning.m).
