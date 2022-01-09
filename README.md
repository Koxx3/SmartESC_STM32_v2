This is an alternative firmware for Xiaomi M365 and Ninebot G30 controller.

It uses VESC interface to communication throught BMS UART (full duplex UART).

Nota : this firmware is in beta. It's not meant to be used with native M365 display for now, and you'll loose the ability to monitor M365 BMS.

You'll be able to setup and control the controller/motor from VESCTool interface with a simple USB/Serial adapter and, with any small arduino, use analog acceleration/brake throttles to control a moving device like escooter, gokart, electric skateboard.

# Build

For an easier build, you need `git` and `docker`.

## Clone the project
`git clone https://github.com/Koxx3/SmartESC_STM32_v2.git`

## Build on Linux
Launch from terminal:

`chmod +x docker_build*; ./docker_build_m365.sh`

`chmod +x docker_build*; ./docker_build_g30.sh`

## Build on Windows
Double click :

`docker_build_m365.sh`

`docker_build_g30.sh`


# Programming

You need a ST-Link device to reprogram the M365/G3O controller.
It costs 3/4€ on Aliexpress.

Plug the st-link following this schematic :
![image](https://user-images.githubusercontent.com/11454444/146688635-b5a1ed07-3482-420f-b324-9e58b0a19dc9.png)

With [STM32 ST-Link Utility](https://www.st.com/en/development-tools/stsw-link004.html), disable Re&d out protection.

Menu "Target" => "Option bytes"
![image](https://user-images.githubusercontent.com/11454444/146688019-3e5122c7-f3fb-4964-a44f-684af023746e.png)


# VescTool

Use [VescTool](https://vesc-project.com/vesc_tool) to setup the motor and input properties.

Use a serial USB adapter to connect the Xiaomi controller as an USB VESC :
![image](https://user-images.githubusercontent.com/11454444/146688647-e3e4d833-7c93-4b4b-a297-cc61ba52071e.png)

Launch VESCTool and connect with COM port.
![image](https://user-images.githubusercontent.com/11454444/146687240-e393ea2e-dfd9-4fac-870e-4cf526a61187.png)

Launcher Motor setup wizzard.
![image](https://user-images.githubusercontent.com/11454444/146688494-b4a6c183-a89f-4517-af1f-61b5358aad40.png)

Enter all your settings in the different windows.

Enable the keyboard control :

![image](https://user-images.githubusercontent.com/11454444/146688470-adf8a8f7-e3b4-43f4-9038-479d3d5585c5.png)

You're ready to test your M365 controller with your keyboard !


# ESP32 test module

M365 connections :
![image](https://user-images.githubusercontent.com/11454444/146688619-c3bc8e6d-6884-4b1c-81d6-9ec456d1e41b.png)

Use ESP32 prototype board with and ESP32-devkit-c module :
![image](https://user-images.githubusercontent.com/11454444/146688428-d8978339-fab1-4a7b-a88f-305298b6b64f.png)

Use the code provided in the [serial-trottle-brake-esp32](/serial-trottle-brake-esp32) folder with Platform.io

# Error Code SESC (Smart ESC) in VESC Tool

Can read it in "VESC Terminal" (or others Serial Terminal)

- 0 = MC_NO_ERROR     (No error)
- 0= MC_NO_FAULTS     (No error)
- 1 = MC_FOC_DURATION (FOC rate to high)
- 2 = MC_OVER_VOLT    (Software over voltage)
- 4 = MC_UNDER_VOLT   (Software under voltage)
- 8 = MC_OVER_TEMP    (Software over temperature)
- 10 = MC_START_UP    (Startup failed)
- 20 = MC_SPEED_FDBK  (Speed feedback)
- 40 = MC_BREAK_IN    (Emergency input (Over current))
- 80 = MC_SW_ERROR

# Command available in Terminal :

- foc_openloop [current] [erpm]  (Exemple : foc_openloop 10 500)
