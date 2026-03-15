# VERSLAG – Opdracht 2: IR Receiver (RC5) – LaserTag Game

Opleiding: Bachelor Elektronica–ICT (Brugge)  
Vak: Game Tech – Academiejaar 2026–2027  
Docent: Van Gaever T.  
Student: **[Jouw naam]** – **[Klasgroep]** – **[Datum]**

---

## 1. Doelstelling
In deze opdracht werd een IR-ontvanger opgebouwd met een **TSOP4838** (38 kHz demodulator) en een **STM32L432KC**. Het doel is het ontvangen van RC5-frames (Manchester-codering) en het decoderen naar **adres**, **commando** en **toggle bit**, met debugging via **USART2 (115200 8N1)** in PuTTY.

---

## 2. Hardwareopstelling

### 2.1 Componenten
- STM32 Nucleo-32 met STM32L432KC
- TSOP4838 IR receiver module
- 100 Ω serieweerstand (voeding TSOP)
- 100 nF condensator (ontkoppeling TSOP)
- USB-kabel naar ST-Link (VCP UART)
- Breadboard + jumpers (indien gebruikt)

### 2.2 Aansluitschema (pin mapping)

**TSOP4838 (TO-92 pinout):**
- Pin 1 = OUT → **PA0** (TIM2_CH1 input)
- Pin 2 = GND → **GND**
- Pin 3 = VS → **3.3 V via 100 Ω**
- **100 nF** tussen VS en GND (zo dicht mogelijk bij TSOP)

**UART debugging (ST-Link VCP):**
- USART2 TX → **PA2**
- USART2 RX → **PA15**
- Verbinding PC ↔ board via **CN1 (ST-Link USB)**

### 2.3 Belangrijke signaaleigenschap TSOP4838
De TSOP4838-uitgang is **actief laag en geïnverteerd**:
- **Idle** (geen IR): uitgang **hoog** (≈ 3.3 V)
- **Burst ontvangen**: uitgang **laag** (≈ 0 V)

Dit is belangrijk bij interpretatie van Manchester/RC5 (de decode-bibliotheek houdt hiermee rekening).

### 2.4 Schematisch overzicht (verplicht)
Voeg hier je schema/tekening toe (zender + ontvanger, alle verbindingen).  

- Afbeelding:  
  `![Schematisch overzicht](images/schema_opdracht2.png)`

*(Maak indien nodig een map `images/` en plaats je bestanden daarin.)*

---

## 3. Softwareconfiguratie (STM32CubeMX / CubeIDE)

### 3.1 USART2 (debug)
Configuratie:
- Mode: Asynchronous
- Baud rate: 115200
- Word length: 8 bits
- Parity: None
- Stop bits: 1
- Data direction: RX & TX
- Hardware flow control: None

Doel: `printf()`-debug naar PuTTY.

### 3.2 Timer input capture voor RC5
TIM2 werd gebruikt om flanken op **PA0** te meten (PWM Input / Input Capture concept):
- Metingen van periode en low-time laten toe om per bit de Manchester-overgang te interpreteren.
- Timeout/overflow wordt gebruikt om een onvolledig frame te resetten.

**CubeMX-notities (invullen):**
- Timer: **[TIM2]**
- Ingangspin: **[PA0 – TIM2_CH1]**
- Interrupts enabled: **[ja/nee + welke]**
- Belangrijke prescaler/clock: **[invullen]**

### 3.3 `printf()` omleiding
De printf-uitvoer werd omgeleid naar USART2 zodat frames live in PuTTY zichtbaar zijn. Buffering werd uitgeschakeld zodat tekst onmiddellijk verschijnt.

---

## 4. Resultaten: ontvangen RC5-frames (PuTTY)

### 4.1 PuTTY instellingen
- COM-poort: **[COMx]**
- Snelheid: **115200**
- Databits: **8**
- Parity: **none**
- Stopbits: **1**
- Flow control: **none**

### 4.2 Voorbeeldoutput (minstens 5 frames, verschillende knoppen)
Plak hier exact je output (min. 5 regels) en voeg een screenshot toe.

1. `[RC5] Adres: 0x__ | Commando: 0x__ | Toggle: _`
2. `[RC5] Adres: 0x__ | Commando: 0x__ | Toggle: _`
3. `[RC5] Adres: 0x__ | Commando: 0x__ | Toggle: _`
4. `[RC5] Adres: 0x__ | Commando: 0x__ | Toggle: _`
5. `[RC5] Adres: 0x__ | Commando: 0x__ | Toggle: _`

- Screenshot:  
  `![PuTTY output](images/putty_frames.png)`

**Observaties (kort):**
- Bij herhaald indrukken van dezelfde knop blijft het **adres/commando** gelijk.
- De **toggle bit** wisselt bij “nieuwe” toetsdrukken (zie sectie 6).

---

## 5. Oscilloscoopmeting: envelope-signaal TSOP4838

### 5.1 Meetopstelling en scope-instellingen
- Probe op TSOP4838 **OUT**
- GND clip op **GND**
- 10× probe aanbevolen
- DC coupling
- Trigger: falling edge, threshold ~1.5 V
- Tijdsbasis: 1 ms/div (frame) en 500 µs/div (bitdetail)

### 5.2 Verwacht signaal (theorie)
Theorie (RC5):
- Halve bitperiode: **T ≈ 889 µs**
- Volledige bitperiode: **2T ≈ 1778 µs**
- Totale frame: **≈ 14 bits ≈ 24.9 ms**

### 5.3 Meting 1: volledige RC5-frame + bitlengtes
Voeg een scope-screenshot toe met het volledige frame en meet 3 willekeurige bits.

- Screenshot volledig frame:  
  `![Scope RC5 frame](images/scope_rc5_frame.png)`

**Gemeten bitlengtes (3 willekeurige bits):**

| Bit # | Gemeten bitperiode (µs) | Theorie (µs) | Afwijking (µs) | Afwijking (%) |
|------:|--------------------------:|-------------:|---------------:|--------------:|
| 1     | [____]                    | 1778         | [____]         | [____]        |
| 2     | [____]                    | 1778         | [____]         | [____]        |
| 3     | [____]                    | 1778         | [____]         | [____]        |

**Bespreking afwijking**  
De afwijking kan veroorzaakt worden door:
- toleranties van de afstandsbediening (oscillator),
- jitter door demodulatie/filtering in TSOP4838,
- trigger/meetcursor onnauwkeurigheid,
- ruis of omgevingslicht,
- (afhankelijk van implementatie) timingvariatie door interrupts.

### 5.4 Meting 2: startbit, field bit en toggle bit identificeren
- **Startbit (S):** altijd logisch ‘1’ en staat aan het begin van het frame.
- **Field bit (F):** bepaalt commando-veld.
- **Toggle bit (C):** wisselt bij elke nieuwe toetsdruk.

**Aanduiding in je scopebeeld (invullen + annoteren):**
- Waar start frame: **[beschrijf]**
- Bitvolgorde: `S | F | C | A4..A0 | C5..C0`
- Screenshot met annotaties:  
  `![Annotated scope](images/scope_annotated_bits.png)`

---

## 6. Toggle bit: verandert die? Wat is het nut?

**Verandert de toggle bit?**  
- Ja/Nee: **[invullen]**
- Bewijs (2 regels met zelfde adres/commando, andere toggle):
  - `[RC5] Adres: 0x__ | Commando: 0x__ | Toggle: 0`
  - `[RC5] Adres: 0x__ | Commando: 0x__ | Toggle: 1`

**Nut van de toggle bit:**
- Onderscheid maken tussen “knop blijft ingedrukt / repeats” en “nieuwe toetsdruk”.
- In toepassingen (zoals LaserTag) kan je hiermee bv. één schot per druk detecteren, en herhalingen negeren of apart behandelen.

---

## 7. Problemen en oplossingen (verplicht)
Beschrijf kort wat misliep en hoe je het oploste. Voorbeelden (pas aan naar jouw situatie):

- **Geen ontvangst / random pulses** → TSOP4838 verkeerd gepind (TO-92 pinout) of ontbrekende 100 nF ontkoppeling; opgelost door correcte aansluiting + RC-filter.
- **Onstabiele decoding** → verkeerde timerconfig/edge polarity; opgelost door correcte input-capture/PWM-input instellingen en juiste interrupt handling.
- **Geen tekst in PuTTY** → verkeerde COM-poort/baud rate of printf niet omgeleid; opgelost door USART2 115200 en `__io_putchar`/`_write`.
- **Frames vallen weg** → te strenge timeout of ruis; opgelost door timeout correct te zetten en bedrading kort te houden.

**Jouw problemen/oplossingen (invullen):**
- [Probleem 1] → [Oplossing]
- [Probleem 2] → [Oplossing]

---

## 8. Conclusie
De IR-receiver met TSOP4838 en STM32L432KC ontvangt RC5-frames **[betrouwbaar/af en toe]**. Via UART-debug werden adres/commando/toggle zichtbaar gemaakt. Scope-metingen bevestigen dat de gemeten bitperioden rond de theoretische **1778 µs** liggen, met beperkte afwijkingen door toleranties en demodulatie. De toggle bit wisselt zoals verwacht en is bruikbaar om nieuwe toetsdrukken van repeats te onderscheiden.

---

## Bijlagen
- Bijlage A: Schema/tekening hardware
- Bijlage B: PuTTY screenshots (min. 5 frames)
- Bijlage C: Oscilloscoop screenshots + annotaties
