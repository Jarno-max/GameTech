# RC5 IR Transmitter voor LaserTag - Fase 1

## Project Overzicht
Dit project implementeert een RC5 IR transmitter op de STM32L432KC microcontroller voor de LaserTag opdracht.

## Hardware Configuratie

### Pinout
- **PA2 (TIM15_CH1)**: IR LED output (via transistor driver)
- **PA1**: Push button input (met interne pull-up)
- **PB3**: LED3 (status indicator)

### IR LED Driver Circuit
Sluit de IR LED aan volgens AN4834 Figure 8:

```
PA2 ---[1kΩ]--- Base (2N2222 NPN transistor)
                Collector --- [IR LED] --- [10Ω resistor] --- VCC (5V)
                Emitter --- GND
```

### Button Circuit
```
PA1 --- [Button] --- GND
(interne pull-up geactiveerd)
```

## Timer Configuratie

### TIM15 - Bit Timing (889μs periodes)
- **Functie**: Genereert de RC5 bit timing
- **Frequentie**: 1125 Hz (889μs periode)
- **Configuratie**:
  - Prescaler: 31 (32MHz / 32 = 1MHz)
  - Period: 889 (889μs)
  - Interrupt: Enabled

### TIM16 - 38kHz Carrier
- **Functie**: Genereert de 38kHz draaggolf met 25% duty cycle
- **Frequentie**: 38.005 kHz
- **Configuratie**:
  - Prescaler: 0 (32MHz clock)
  - Period: 841 (842 ticks)
  - PWM Pulse: 210 (≈25% duty cycle)
  - Output: PA2 via forced output compare

## RC5 Protocol

### Frame Structuur (14 bits)
```
| S1 | S2 | C | A4 | A3 | A2 | A1 | A0 | I5 | I4 | I3 | I2 | I1 | I0 |
```

- **S1**: Start bit 1 (altijd 1)
- **S2**: Start bit 2 / Field bit
- **C**: Toggle/Control bit (wisselt bij elke druk)
- **A[4:0]**: Address (5 bits, 0-31)
- **I[5:0]**: Command (6 bits, 0-63)

### Manchester Encoding
- **Logic 0**: 01 (short low, short high)
- **Logic 1**: 10 (short high, short low)
- Elke bit duurt 1.778ms (2 × 889μs)

### Standaard Configuratie
- **Address**: 1 (apparaat ID)
- **Command**: 12 (Play/Pause)
- **Toggle bit**: Wisselt automatisch bij elke verzending

## Gebruik

1. **Compileer en upload** het project naar de STM32L432KC
2. **Hardware setup**: Sluit de IR LED driver aan op PA2
3. **Button**: Sluit een drukknop aan tussen PA1 en GND
4. **Power on**: LED3 knippert kort om aan te geven dat het systeem klaar is
5. **Test transmissie**: 
   - Druk op de button
   - LED3 licht op tijdens verzending
   - IR LED zendt RC5 frame uit

## Oscilloscoop Metingen

### 1. 38kHz Draaggolf
**Probe op PA2:**
- **Frequentie**: ~38 kHz
- **Periode**: 26.3 μs
- **Duty cycle**: ~25%
- **High tijd**: ~6.6 μs
- **Low tijd**: ~19.7 μs

### 2. RC5 Manchester Frame
**Probe op PA2 met lange timebase (10ms/div):**

Compleet frame structuur (één button druk):
```
|<---- ~25ms totale frame tijd ---->|

S1  S2   C  A4  A3  A2  A1  A0  I5  I4  I3  I2  I1  I0
[1] [1] [0][0] [0] [0] [0] [1][1] [1] [0] [0] [0] [0]
===---===---==--=---=---=---=---===--===---==--==--==--

Manchester bits:
'1' = high-low (===---)
'0' = low-high (---===)

Elke bit: 1.778ms (889μs × 2)
Totaal: 14 bits × 1.778ms = 24.9ms
```

### 3. Bit Details (zoom in op enkele bits)
**Timebase: 500μs/div**
```
Manchester '1':     Manchester '0':
    ____               
   |    |___         ___    ____
   |<-889μs->|      |<-889μs->|
   |<--1.778ms->|   |<--1.778ms->|
```

### 4. 38kHz Burst Detail
**Zoom op een high periode:**
- Je ziet de 38kHz oscillatie tijdens de high periodes
- Low periodes: geen oscillatie (carrier uit)

## Oscilloscoop Settings voor Metingen

### Meting 1: Carrier Frequentie
- **Timebase**: 10 μs/div
- **Voltage**: 1V/div
- **Trigger**: Rising edge, auto
- **Meet**: Frequentie, duty cycle, periode

### Meting 2: Manchester Frame
- **Timebase**: 2 ms/div
- **Voltage**: 2V/div
- **Trigger**: Rising edge, single shot
- **Action**: Druk op button terwijl trigger wacht

### Meting 3: Bit Timing
- **Timebase**: 500 μs/div
- **Voltage**: 2V/div
- **Cursor**: Meet 889μs per half-bit

## Verificatie Checklist

- [ ] 38kHz carrier zichtbaar op oscilloscoop
- [ ] 25% duty cycle gemeten
- [ ] Manchester encoding correct (1=10, 0=01)
- [ ] Bit tijd = 1.778ms per bit
- [ ] Frame totaal ~25ms
- [ ] LED3 knippert bij button druk
- [ ] IR LED zendt (zichtbaar met smartphone camera)

## Code Structuur

### Bestanden
```
Core/
├── Inc/
│   ├── ir_common.h        # Timer en protocol definities
│   ├── rc5_encode.h       # RC5 encoder header
│   └── main.h             # Hoofdheader
└── Src/
    ├── rc5_encode.c       # RC5 encoder implementatie
    ├── main.c             # Main programma met button handling
    └── stm32l4xx_it.c     # Interrupt handlers
```

### Belangrijke Functies

#### RC5_Encode_Init()
Initialiseert beide timers:
- TIM15: Output compare voor bit timing
- TIM16: PWM voor 38kHz carrier

#### RC5_Encode_SendFrame(address, command, ctrl)
Stuurt een RC5 frame:
1. Genereert binary frame
2. Converteert naar Manchester
3. Start TIM15 interrupt
4. Elke interrupt: volgende bit versturen

#### RC5_Encode_SignalGenerate()
Wordt aangeroepen vanuit TIM15 interrupt:
- Leest volgende bit uit Manchester frame
- Zet carrier aan (bit=1) of uit (bit=0)
- Stopt na alle bits verzonden

## Debug Tips

### LED knippert niet
- Check button aansluiting op PA1
- Verifieer EXTI1 interrupt enabled
- Breakpoint in `HAL_GPIO_EXTI_Callback()`

### Geen signaal op PA2
- Check TIM15/TIM16 configuratie
- Verifieer `RC5_Encode_Init()` wordt aangeroepen
- Check clock frequencies (32MHz?)

### Signaal incorrect
- Verifieer timer periods:
  - TIM15: 889 (bit timing)
  - TIM16: 841 (38kHz carrier)
- Check Manchester conversie

## Timing Berekeningen

### 38kHz Carrier (TIM16)
```
f_TIM = 32 MHz (na prescaler = 0)
f_carrier = 38 kHz
Period = f_TIM / f_carrier = 32,000,000 / 38,000 = 842 (ARR = 841)
Duty 25% = 842 / 4 = 210 (CCR1)
```

### 889μs Bit Timing (TIM15)
```
f_TIM = 32 MHz
Prescaler = 31 → f_TIM = 32MHz / 32 = 1 MHz
Period = 889 → t = 889μs
Half-bit = 889μs
Full bit = 1778μs (RC5 spec: 1.778ms ✓)
```

## Volgende Stappen (Fase 2)

In de volgende fase voeg je toe:
- IR receiver (TSOP4838)
- RC5 decoder
- Hit detectie
- Score systeem

## Referenties

- **AN4834**: STM32 infrared (IR) remote module firmware
- **RC5 Protocol**: Philips RC-5 specification
- **STM32L432KC**: Reference Manual RM0394

---
**Auteur**: Game Tech Project  
**Datum**: 2026  
**Versie**: 1.0 (Fase 1)
