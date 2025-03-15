TEAM 2 DART FIRMWARE DEVELOPMENT

Authors: Abhirit Das, Ben Park, Lucas Sanabria, Jason Kim

Progress:
1) Operate an LED (10.27.2024)
2) Able to sample IMU data using SPI and transmit data through UART (11.16.2024)
3) Successful Mahony Filter on IMU data to estimate pitch, roll, and yaw angles
4) Sucessfully retrieved altitude and temperature data from barometer (will likely need some reevaluation)

Current Goals:
1) Develop modular code to interface with servos and motors
2) Implement basic altitude PID control in firmware and perform HIL on pivot arm test rig
3) Design a high-level control system model in Python
4) Write control system in firmware...

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

        
        



