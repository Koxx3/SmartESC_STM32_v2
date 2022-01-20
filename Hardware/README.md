# Hardware Tools

Mosfets bending tool M365 to easily bend the legs of new mosfets 

Link : 
- Fusion360 https://a360.co/3qcQ0Wm
- STL [Bender legs TO220 M365 ESC] in the /hardware_tools folder


# Hardware Informations to Upgrade

Informations for Upgrade the Hardware 
- Google Sheet : https://docs.google.com/spreadsheets/d/1jOdUUpbCZVHQc2-eIezrbkNFHuf14I3zOeL0la91yoo/edit?usp=sharing


Tutorial for Simple 20S modification 

- Hardware modification : https://docs.google.com/spreadsheets/d/15NJW5ABOqHJWt1sHm2iTJhP_wqReokJQz8fKPkYVTDk/edit?usp=sharing

- Software :
- 
For match the Hardware and the Software you have to set VOLTAGE_DIVIDER_GAIN line 51 in the product.h at "1480" with CubeIDE

Warning
the Main voltage Regulator is limited at 90v ABSOLUT MAX, you need to select the maximum voltage protection in VESC Tool at 86~88v maximum, and be carful with the electric brake and the current when the 20S battery is full charged, i recommand to dont charge the battery at maximum voltage, something like 82-83v should be fine

you can see the road here : 

![Image](https://github.com/Koxx3/SmartESC_STM32_v2/blob/vesc_comp/Hardware/Images/CubeIDE.png)

![Image](https://github.com/Koxx3/SmartESC_STM32_v2/blob/vesc_comp/Hardware/Images/CubeIDE2.png)





by Foujiwara
