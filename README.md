# Dashboard

## Analisi Dashboard SC19

La scheda può essere suddivisa in 4 sezioni:

- MCU (STM32)
- Input/Output (Connettore)
- Alimentazione (DC/DC)
- Condizionamento Output/Input

### MCU (STM32)
Il microutilizzato è un **STM32F303VCT**. 
- Alimentato con una tensione di 3.3V (ricavata tramite modulo DC/DC sulla scheda)
- Collegamento CAN con transreceiver SN65HVD234
- Connettore SWD
- Tre LED sulla scheda, RED/GREEN/BLUE, collegati rispettivamente ai pin PE11, PE12, PA3 e comandati tramite MOSFET (sulla scheda)
- GPIO (verranno spiegati in seguito)
  - PD10 : Imd_Err_Led_Cmd
  - PE2: Buzzer
  - PE3: No_Hv_Led
  - PE5: Rtd_Led
  - PD13: PWM1
  - PD14: PWM2
  - PD15: PWM3
  - PA1: spare_button
  - PA2: RTD_button
  - PB9: BMS_Led_Cmd
- I2C
- USART
### Input/Output (Connettore)
### Alimentazione (DC/DC)
### Condizionamento Output/Input

