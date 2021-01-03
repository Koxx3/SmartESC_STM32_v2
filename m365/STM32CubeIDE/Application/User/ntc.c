/*
* Die NTC Tabelle, bestehend aus 33 Temperaturstützpunkten.
* Einheit:0.1 °C
*/

int NTC_table[33] = {
  1835, 1493, 1151, 969, 847, 754, 678, 615,
  560, 511, 466, 425, 386, 350, 316, 282, 250,
  218, 187, 156, 125, 93, 61, 28, -6, -43,
  -83, -126, -176, -235, -311, -428, -545
};


/*
* \brief    Konvertiert das ADC Ergebnis in einen Temperaturwert.
*
*           Mit p1 und p2 wird der Stützpunkt direkt vor und nach dem
*           ADC Wert ermittelt. Zwischen beiden Stützpunkten wird linear
*           interpoliert. Der Code ist sehr klein und schnell.
*           Es wird lediglich eine Ganzzahl-Multiplikation verwendet.
*           Die Division kann vom Compiler durch eine Schiebeoperation
*           ersetzt werden.
*
*           Im Temperaturbereich von -10°C bis 200°C beträgt der Fehler
*           durch die Verwendung einer Tabelle 30.803°C
*
* \param    adc_value  Das gewandelte ADC Ergebnis
* \return              Die Temperatur in 0.1 °C
*
*/

int NTC_ADC2Temperature(unsigned int adc_value){

  int p1,p2;
  /* Stützpunkt vor und nach dem ADC Wert ermitteln. */
  p1 = NTC_table[ (adc_value >> 7)  ];
  p2 = NTC_table[ (adc_value >> 7)+1];

  /* Zwischen beiden Punkten linear interpolieren. */
  return p1 - ( (p1-p2) * (adc_value & 0x007F) ) / 128;
};


/*
  Copyright (C) Preis Ingenieurbüro GmbH
  Licensed under the Apache License, Version 2.0 (the "License").
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
  http://www.apache.org/licenses/LICENSE-2.0
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/
