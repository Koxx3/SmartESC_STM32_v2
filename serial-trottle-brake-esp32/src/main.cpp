#include <Arduino.h>
#include <VescUart.h>

#define DEBUG 1
#define SERIAL_BAUD 921600 // debug baudrate

#define BAUD_RATE_SMARTESC_XIAOMI 115200 // default baudrate of UART on SmartESC

// pin layout
#define PIN_SERIAL_ESP_TO_CNTRL 27 // TX esp32 -> RX controller
#define PIN_SERIAL_CNTRL_TO_ESP 14 // RX esp32 <- TX controller
#define PIN_IN_ABRAKE 34           // Brake
#define PIN_IN_ATHROTTLE 39        // Throttle

// send orders every 40ms
#define TIME_SEND 40

// deadband for analog input (applied to analog raw range 0-4095)
#define ANALOG_SAFETY_OFFSET 100

// analog min/max RAW value (range 0-4095)
// need to be set according to your ADC input voltage divider
// 3.3V = 4095
#define ANALOG_MIN_RAW 600
#define ANALOG_MAX_RAW 3200

// minimal ADC RAW value for initial calibration
// if adc read are higher, it use default minimal calibration
#define MINIMAL_ANALOG_VALUE 30

// Trottle
uint16_t analogValueThrottleRaw = 0;
uint16_t analogValueThrottleMinCalibRaw = 0;

// Brake
uint16_t analogValueBrakeRaw = 0;
uint16_t analogValueBrakeMinCalibRaw = 0;

char print_buffer[500];

// Initiate VescUart class
VescUart vescCntrl;

// hardware serial declaration
HardwareSerial hwSerCntrl(1);

// last order send time
unsigned long iTimeSend = 0;

// ########################## SETUP ##########################
void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("SmartESC Serial v1.0 - vesc interface");

  pinMode(PIN_IN_ATHROTTLE, INPUT);
  pinMode(PIN_IN_ABRAKE, INPUT);

  // do it twice to improve values
  uint16_t adcValue = 0;
  adcValue = analogRead(PIN_IN_ATHROTTLE);
  adcValue = analogRead(PIN_IN_ATHROTTLE);
  if (adcValue < ANALOG_MIN_RAW)
    analogValueThrottleMinCalibRaw = adcValue;
  else
    analogValueThrottleMinCalibRaw = ANALOG_MIN_RAW;

  analogValueBrakeMinCalibRaw = analogRead(PIN_IN_ABRAKE);
  analogValueBrakeMinCalibRaw = analogRead(PIN_IN_ABRAKE);
  if (adcValue < ANALOG_MIN_RAW)
    analogValueThrottleMinCalibRaw = adcValue;
  else
    analogValueThrottleMinCalibRaw = ANALOG_MIN_RAW;

  hwSerCntrl.begin(BAUD_RATE_SMARTESC_XIAOMI, SERIAL_8N1, PIN_SERIAL_CNTRL_TO_ESP, PIN_SERIAL_ESP_TO_CNTRL);

  while (!hwSerCntrl)
  {
    ;
  }

  /** Define which ports to use as UART */
  vescCntrl.setSerialPort(&hwSerCntrl);
}

// ########################## LOOP ##########################

void loop(void)
{
  unsigned long timeNow = millis();

  // Avoid delay
  if (iTimeSend > timeNow)
    return;
  iTimeSend = timeNow + TIME_SEND;

  // Compute throttle
  analogValueThrottleRaw = analogRead(PIN_IN_ATHROTTLE);
  analogValueThrottleRaw = constrain(analogValueThrottleRaw, analogValueThrottleMinCalibRaw, ANALOG_MAX_RAW);

  // Compute brake
  analogValueBrakeRaw = analogRead(PIN_IN_ABRAKE);
  analogValueBrakeRaw = constrain(analogValueBrakeRaw, analogValueBrakeMinCalibRaw, ANALOG_MAX_RAW);

  // The valueY is used to control the speed, where 127 is the middle = no current
  uint8_t value;
  if (analogValueBrakeRaw > analogValueBrakeMinCalibRaw + ANALOG_SAFETY_OFFSET)
    value = map(analogValueBrakeRaw, analogValueBrakeMinCalibRaw + ANALOG_SAFETY_OFFSET, ANALOG_MAX_RAW, 126, 0);
  else if (analogValueThrottleRaw > analogValueThrottleMinCalibRaw + ANALOG_SAFETY_OFFSET)
    value = map(analogValueThrottleRaw, analogValueThrottleMinCalibRaw + ANALOG_SAFETY_OFFSET, ANALOG_MAX_RAW, 127, 255);
  else
    value = 127;

#if DEBUG
  Serial.println("throttleRaw = " + (String)analogValueThrottleRaw + " / throttleMinCalibRaw = " + (String)analogValueThrottleMinCalibRaw +
                 " / brakeRaw = " + (String)analogValueBrakeRaw + " / brakeMinCalibRaw = " + (String)analogValueBrakeMinCalibRaw +
                 " / nunchuckValue = " + (String)value);
#endif

  // Call the function setNunchuc kValues to send the current nunchuck values to the VESC
  vescCntrl.nunchuck.valueY = value;
  vescCntrl.setNunchuckValues();
}

// ########################## END ##########################
