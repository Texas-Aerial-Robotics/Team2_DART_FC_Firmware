TEAM 2 DART FIRMWARE DEVELOPMENT

Authors: Abhirit Das, Ben Park, Lucas Sanabria, Jason Kim

Progress:
1) Operate an LED (10.27.2024)
2) Able to sample IMU data using SPI and transmit data through UART (11.16.2024)

Current Goals:
1) Apply filters for stable angle and altitude values
2) Design a high-level control system model
3) Implement PID control in firmware
4) Evaluate ways to test firmware on a physical DART prototype

Documentation:

Install STM32CubeIDE 
https://www.st.com/en/development-tools/stm32cubeide.html#get-software

Install STM32H7 Firmware
In CubeIDE go to 
Help > Configuration Tools > Manage Embedded Software Packages
![image](https://github.com/user-attachments/assets/f08cfbec-0236-49e6-85c7-666b6fda1644)

Common Problems:
    -"float formating support is not enabled" when printing a float through SPI
      To fix turn on these two settings
        right click on the current project > properties > C/C++ Build > MCU/MPU Settings
        check these two boxes
        ![image](https://github.com/user-attachments/assets/bb0ee3e6-e83a-43b7-bfc3-b714da603ac4)

        
        



