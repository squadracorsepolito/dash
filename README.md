![Dash](dash.png)
Benvenuti nella repository ufficiale certificata contenente i file utilizzati per la scheda NuovaDash22© utilizzata dal team Squadra Corse Driverless nell'anno 2021/22

Repository mantenuta da Matteo Bonora (Software) e Francesco Minichelli (Hardware e Software)

### Configurazione STM32CubeIDE per upload software tramite CAN

Project Properties -> C/C++ Build -> Settings -> Tool Settings -> MCU Post build outputs -> Selezionare "Convert to Motorola S-record file (-O ihex)"

### Run this command on the terminal to upload the code

1) Setup CAN0 interface with a Kvaser device connected to the PC Host
2a) Open MicroBoot
2b) Open BootCommander (same as MicroBoot but on a terminal)
3) Make sure the Target board is off and the can bus of the board connected to the Kvaser
  - There should be no problem if other device are connected on same can bus. If problem arises, turn off all other devices.
4a) Setup MicroBoot with the firmware to upload. 
4b) Lunch the terminal command
5) Power on the Target board.

```shell
cd /home/francesco/Desktop/openblt-master/Host
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
./MicroBoot
```

La configurazione attuale è la seguente
CAN SPEED = 1Mbit
CAN message ID target-\>host = (0xFFA)
CAN message ID host-\>target = (0x007)

## Analisi NuovaDash22

I Micro attualmente utilizzati sono:
- STM32F303VCTx (256Kb version)
- STM32F303VETx (512Kb version)

Le periferiche attualmente utilizzate sono:
- CAN
- GPIO

Di seguito una descrizione (tensione, impedenza) per ogni pin del connettore in base all'uscita del micro.

| Signal Name  	| GPIO PIN SET 	 | GPIO PIN RESET | PIN NAME |
| ------------- | -------------- | -------------- | -------- |
| ASB CMD  	| ASB LED ON  	 | ASB LED OFF	  | PA8      |
| AMS CMD  	| AMS LED ON  	 | AMS LED OFF    | PC6      |
| IMD CMD  	| IMD LED ON  	 | IMD LED OFF    | PC8      |
| RTD CMD  	| RTD LED ON  	 | RTD LED OFF    | PC6      |
| TSOFF CMD  	| TSOFF LED ON 	 | TSOFF LED OFF  | PC11     |
| ASSI B CMD  	| ASSI B LED OFF | ASSI B LED ON  | PB8      |
| ASSI Y CMD  	| ASSI Y LED OFF | ASSI Y LED ON  | PB7      |
| AMI 1 CMD  	| AMI 1 LED OFF  | AMI 1 LED ON   | PB5      |
| AMI 2 CMD  	| AMI 2 LED OFF  | AMI 2 LED ON   | PD6      |
| AMI 3 CMD  	| AMI 3 LED OFF  | AMI 3 LED ON   | PD4      |


Appena la scheda parte, sia durante il bootloader che durante le prime fasi del programma (1 secondo di bootloader + [0-2.5 max] secondi di programma), al fine di rispettare la regola T11.9.5 che cito: 

> T 11.9.5 Indicators according to T 11.9.1 with safe state “illuminated” (e.g. absence of failures is not actively indicated) must be illuminated for 1 s to 3 s for visible check after power cycling the LVMS. 

ci si aspetta il seguente stato per ogni apparato della macchina:

- LED ASSI YELLOW (durante i test questo colore sarà BLU)
- LED AMI tutti accesi
- BuzzerAS Spento
- BuzzerEv Spento
- Cockpit tutto acceso tranne TSOFF


### Analisi Dashboard SC19

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

