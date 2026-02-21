# Color-Based-product-sorting-Conveyor

This repository contains the firmware for an automated color sorting conveyor powered by an STM32F103C8T6 microcontroller. The system operates by utilizing an infrared obstacle sensor attached to an external interrupt (EXTI) pin to detect incoming objects and track the total product count. Once an object is in position, a TCS3200 color sensor reads its RGB signature.  

The STM32 accurately processes this data by using hardware timer input capture to measure the sensor's output pulse widths across different color filters. Based on the calibrated RGB thresholds, the object is classified into one of four categories: orange, green, yellow, or error (blue and pink). The microcontroller then generates precise PWM signals to coordinate three MG90S servo motors, which handle the feeding mechanism, rotate the sorting chute to the designated angle, and drop the item into the correct bin. The number of each products are then displayed o a 16x2 I2C LCD for easy monitoring.

To deploy this project, simply clone the repository, open it in STM32CubeIDE, and flash the compiled code to your board. Ensure that the servo PWM timer and the input capture timer for the TCS3200 are correctly mapped to your hardware setup before powering on the system.
