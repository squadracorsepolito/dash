# Dashboard

Benvenuti nella repository ufficiale contente i file utilizzati per la scheda NuovaDash22 utilizzata dal team Squadra Corse Driverless nell'anno 2021/22

Repository mantenuta da Matteo Bonora (Software) e Francesco Minichelli (Hardware e Software)

### Configurazione STM32CubeIDE per upload software tramite CAN

Project Properties -> C/C++ Build -> Settings -> Tool Settings -> MCU Post build outputs -> Selezionare "Convert to Motorola S-record file (-O ihex)"

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
  - PD10 : Imd\_Err\_Led\_Cmd
  - PE2: Buzzer\_CMD
  - PE3: No\_Hv\_Led\_CMD
  - PE5: Rtd\_Led\_CMD
  - PD13: PWM1
  - PD14: PWM2
  - PD15: PWM3
  - PA1: spare\_button
  - PA2: RTD\_button (RTD\_In)
  - PB9: BMS\_Led\_Cmd
- I2C
- USART
### Input/Output (Connettore)
### Alimentazione (DC/DC)
Input a 24V e output a 5V e 3.3V. Per la 5V è stato utilizzato un LMZ14202TZE e per la 3.3V un LM1117-3.3
### Condizionamento Output/Input
I seguenti segnali, provenienti dal connettore, sono collegati a un MOSFET con Source collegato a massa. Tra parentesi, il nome del GPIO della STM che controlla il mosfet
- IMD\_LED (IMD\_LED\_CMD)
- BMS\_LED (BMS\_LED\_CMD)
- RTD\_LED (RTD\_LED\_CMD)
- BUZZER (BUZZER\_CMD)
- No\_Hv\_Led (No\_Hv\_Led\_CMD)
- PUMP\_AUTO (PWM3)
- BP\_FAN\_AUTO (PWM2)
- RAD\_FAN\_AUTO (PWM1)

