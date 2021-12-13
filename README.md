# Dashboard

## Analisi Dashboard SC19

La scheda può essere suddivisa in 4 sezioni:

- MCU (STM32)
- Input/Output (Connettore)
- Alimentazione (DC/DC)
- Condizionamento Output/Input

### MCU (STM32)
Il micro utilizzato è un **STM32F303VCT**. 
- Alimentato con una tensione di 3.3V (ricavata tramite modulo DC/DC sulla scheda)
- Collegamento CAN con transreceiver SN65HVD234
- Connettore SWD
- Tre LED sulla scheda, RED/GREEN/BLUE, collegati rispettivamente ai pin PE11, PE12, PA3 e comandati tramite MOSFET (sulla scheda)
- GPIO (verranno spiegati in seguito)
  - PD10 : Imd_Err_Led_Cmd
  - PE2: Buzzer_CMD
  - PE3: No_Hv_Led_CMD
  - PE5: Rtd_Led_CMD
  - PD13: PWM1
  - PD14: PWM2
  - PD15: PWM3
  - PA1: spare_button
  - PA2: RTD_button (RTD_In)
  - PB9: BMS_Led_Cmd
- I2C
- USART
### Input/Output (Connettore)
### Alimentazione (DC/DC)
Input a 24V e output a 5V e 3.3V. Per la 5V è stato utilizzato un LMZ14202TZE e per la 3.3V un LM1117-3.3
### Condizionamento Output/Input
I seguenti segnali, provenienti dal connettore, sono collegati a un MOSFET con Source collegato a massa. Tra parentesi, il nome del GPIO della STM che controlla il mosfet
- IMD_LED (IMD_LED_CMD)
- BMS_LED (BMS_LED_CMD)
- RTD_LED (RTD_LED_CMD)
- BUZZER (BUZZER_CMD)
- No_Hv_Led (No_Hv_Led_CMD)
- PUMP_AUTO (PWM3)
- BP_FAN_AUTO (PWM2)
- RAD_FAN_AUTO (PWM1)

