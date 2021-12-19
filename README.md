This is an alternative firmware for Xiaomi M365 and Ninebot G30 controller

It uses VESC interface to communication throught BMS UART (full duplex UART).

Nota : this firmware is in beta. It's not meant to be used with native M365 display for now, and you'll loose the ability to monitor M365 BMS.

You'll be able to setup and control the controller/motor from VESCTool interface with a simple USB/Serial adapter and, with any small arduino, use analog acceleration/brake throttles to control a moving device like escooter, gokart, electric skateboard.

# Programming

You need a ST-Link device to reprogram the M365/G3O controller.
It costs 3/4€ on Aliexpress.

Plug the st-link following this schematic :
![image](https://user-images.githubusercontent.com/11454444/146687936-fecefada-6a42-4906-8ce2-6a6d8b60b813.png)

With 'STM32 ST-Link Utility'(https://www.st.com/en/development-tools/stsw-link004.html), disable Red out protection.

Menu "Target" => "Option bytes"
![image](https://user-images.githubusercontent.com/11454444/146688019-3e5122c7-f3fb-4964-a44f-684af023746e.png)


# VescTool

Use VescTool(https://vesc-project.com/vesc_tool) to setup the motor and input properties.

Use a serial USB adapter to connect the Xiaomi controller as an USB VESC :
![image](https://user-images.githubusercontent.com/11454444/146688078-1a626e9a-ee29-491e-ae80-c56765fcdf11.png)

Launch VESCTool and connect with COM port.
![image](https://user-images.githubusercontent.com/11454444/146687240-e393ea2e-dfd9-4fac-870e-4cf526a61187.png)

Launcher Motor setup wizzard.
Enable keyboard control in the right toolbar :
![image](https://user-images.githubusercontent.com/11454444/146688140-10e621de-3ac7-4234-8a69-d7cb0e2779bc.png)


# ESP32 test module

M365 connections :
![image](https://user-images.githubusercontent.com/11454444/146688214-604f43c4-f962-4c4a-937e-974da73d6f42.png)

Use ESP32 prototype board with and ESP32-devkit-c module :
![image](https://user-images.githubusercontent.com/11454444/146687116-ad32be9d-6a3b-4f04-a995-02352c582d16.png)

Use the code provided in the serial-trottle-brake-esp32 folder with Platform.io

