This document records the naming conventions and abbreviations used in the code. All code should follow this convention. 

## General

- If the variable is a actuator, follow a pattern like `slider_mt`, `slider_sv`, or `slider_lfMt`. Put the actuated part first, then the actuator type. Separate the two parts with an underscore.

- If the variable have a suffix of `adj`, it means that the implementation of this var in Kotlin is through getters and setters. And assigning new value to this var will change the physical state of the actuator. For example `vtSliderHeightAdj`. 

## Definitions

- 

## Abbreviations

- horizontal --> `hz`. Use this in front of the name of the component, for example `hzSlider`.
- vertical --> `vt`. Use this in front of the name of the component, for example `vtSlider` or `hSlider`.
- front --> `fr`. Use this in front of the name of the component, for example `frSlider`.
- back --> `bk`. Use this in front of the name of the component, for example `bkSlider`.
- Left --> `lf`. For example `slider_lfMt`.
- Right --> `rt`. For example `slider_rtMt`.
- motor --> `mt`. For example `slider_mt`.
- Servo --> `sv`. For example `slider_sv`. 
- Specimen --> `sp`