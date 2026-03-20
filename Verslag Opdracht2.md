# VERSLAG – Opdracht 2: IR Receiver (RC5) – LaserTag Game

Opleiding: Bachelor Elektronica–ICT (Brugge)  
Vak: Game Tech – Academiejaar 2026–2027  
Docent: Van Gaever T.  
Student: **[BostynJarno]** – **[20/03/2026]**

---

## 1. Doelstelling
In deze opdracht werd een IR-ontvanger opgebouwd met een **TSOP4838** (38 kHz demodulator) en een **STM32L432KC**. Het doel is het ontvangen van RC5-frames (Manchester-codering) en het decoderen naar **adres**, **commando** en **toggle bit**, met debugging via **USART2 (115200 8N1)** in PuTTY.

---

## 2. Hardwareopstelling

### 2.1 Componenten
- STM32 Nucleo-32 met STM32L432KC (receiver)
- TSOP4838 IR receiver module
- 100 Ω serieweerstand (voeding TSOP)
- 100 nF condensator (ontkoppeling TSOP)
- USB-kabel naar ST-Link (VCP UART)
- Breadboard + jumpers (indien gebruikt)

*(Testopstelling, indien gebruikt / zoals op de foto zichtbaar lijkt):*
- Tweede STM32 Nucleo-32 board als IR-zender/testbron (om RC5/IR-signalen te genereren tijdens testen)

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

### 2.4 Schematisch overzicht
Voeg hier je schema/tekening toe (zender + ontvanger, alle verbindingen).  

- Afbeelding:  
  `![Schematisch overzicht](images/schema_opdracht2.png)`


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
- Timer: **TIM2** (Input Capture op CH1/CH2 voor flankmeting)
- Ingangspin: **PA0 – TIM2_CH1** (AF1)
- Interrupts enabled: **ja** → **TIM2 global interrupt (TIM2_IRQn)** ingeschakeld (priority 0)
- Belangrijke prescaler/clock:
  - TIM2 clock: **16 MHz** (HSI16)
  - Prescaler: **15** ⇒ tellerfrequentie = 16 MHz / (15+1) = **1 MHz** (dus **1 tick = 1 µs**)
  - Period (ARR): **3600** ⇒ overflow na **3.6 ms** (timeout/reset bij geen IR-activiteit)

### 3.3 `printf()` omleiding
De printf-uitvoer werd omgeleid naar USART2 zodat frames live in PuTTY zichtbaar zijn. Buffering werd uitgeschakeld zodat tekst onmiddellijk verschijnt.

### 3.4 Korte code-uitleg (implementatie)
Onderstaand wordt enkel de **kernlogica** toegelicht (geen code dump). Het doel is aantonen dat de ontvangen TSOP4838-envelope correct wordt omgezet naar RC5-bits en uiteindelijk naar **adres/commando/toggle**.

**1) Flankmeting met TIM2 (PA0)**
- TIM2 staat in **reset slave mode** op de TI1-trigger: bij elke **falling edge** (begin van een “burst”, TSOP is actief-laag) wordt de teller gereset.
- **CH1 (falling capture)** meet de tijd tussen twee falling edges (periode fall→fall).
- **CH2 (rising capture, indirect TI)** meet de duur van de **lage puls** (low-time) tussen falling→rising.
- Omdat de RC5-decoder tijdsduren verwacht tussen opeenvolgende Manchester-overgangen (typisch rond **T ≈ 889 µs** of **2T ≈ 1778 µs**), wordt uit de gemeten waarden een bruikbare pulsduur afgeleid:
  - `highPulse = period(fall→fall) - lowPulse(fall→rise)`
  - die `highPulse` komt overeen met de resterende “hoogte”-tijd en vermijdt dat een fall→fall meting soms **3T/4T** lijkt.
- Zowel hardwarematig (IC filter) als softwarematig (minimumduur) worden zeer korte spikes/glitches gefilterd.

**2) TSOP4838 is geïnverteerd (actief-laag)**
- De TSOP-uitgang is **hoog in idle** en trekt **laag** tijdens een IR-burst.
- Daarom worden de gemeten flanken omgezet naar “logische” flanken voor de decoder:
  - echte **falling** wordt behandeld als **logische rising**
  - echte **rising** wordt behandeld als **logische falling**

**3) RC5 bit-decoding op basis van 1T/2T en ‘last bit’**
- In de decoder wordt elke gemeten pulsduur eerst geclassificeerd als **1T** of **2T** binnen toleranties.
- Met een kleine toestandsmachine (via een ‘last bit’ + logica-tabellen) wordt uit de sequentie van flanken de volgende bitwaarde bepaald.
- Zodra alle bits van het frame zijn opgebouwd, wordt een “frame ontvangen”-flag gezet.

**4) Timeout/reset om halve frames te vermijden**
- Bij **geen flanken** binnen ongeveer **3.6 ms** treedt een timer overflow op: het lopende pakket wordt gereset zodat een volgend frame terug proper kan starten.
- Belangrijk detail: de timer-update interrupt wordt zo ingesteld dat die enkel op echte overflow komt (niet op elke slave-reset). Concreet gebeurt dit door **URS=1** te gebruiken, anders zou elk begin van een puls het pakket meteen resetten.

**5) Verwerking in de main loop + UART-output**
- De main loop wacht tot de “frame ontvangen”-flag actief wordt, roept dan de decode-functie op en print:
  - `Adres` (5 bits)
  - `Commando` (6 bits, incl. uitbreiding via field bit)
  - `Toggle` (1 bit)
- Nadien wordt de flag gereset zodat hetzelfde frame niet blijft herprinten.

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

### 5.2 Verwacht signaal (theorie)
Theorie (RC5):
- Halve bitperiode: **T ≈ 889 µs**
- Volledige bitperiode: **2T ≈ 1778 µs**
- Totale frame: **≈ 14 bits ≈ 24.9 ms**

### 5.3 Meting 1: volledige RC5-frame + bitlengtes
Voeg een scope-screenshot toe met het volledige frame en meet 3 willekeurige bits.

- Screenshot volledig frame:  
  ![Scope RC5 frame](<Opdracht2/Scoop Receiver.png>)

*(Op dit screenshot kan je het volledige RC5-frame herkennen. Gebruik cursors/measure om 3 bitperioden te meten en vul ze in in je tabel.)*



**Bespreking afwijking**  
Kleine afwijkingen t.o.v. de theoretische bitduur zijn normaal en kunnen o.a. komen door toleranties van de zender en meetonnauwkeurigheid (cursors/trigger) op de oscilloscoop.

### 5.4 Meting 2: startbit, field bit en toggle bit identificeren
- **Startbit (S):** altijd logisch ‘1’ en staat aan het begin van het frame.
- **Field bit (F):** bepaalt commando-veld.
- **Toggle bit (C):** wisselt bij elke nieuwe toetsdruk.

**Aanduiding in je scopebeeld (invullen + annoteren):**
- Waar start frame: **[beschrijf]**
- Bitvolgorde: `S | F | C | A4..A0 | C5..C0`
- Screenshot (toggle = 0):  
  ![Scope RC5 toggle 0](<Opdracht2/Scoop Reciever Toggle0.png>)

*(Tip: Annoteer in dezelfde afbeelding waar S, F en C staan door de eerste 3 bitposities te markeren. Combineer dit met je gedecodeerde UART-output zodat je zeker bent van de bitvolgorde.)*

---

## 6. Toggle bit: verandert die? Wat is het nut?

**Verandert de toggle bit?**  
- Ja/Nee: **[Ja]**
- Bewijs (2 regels met zelfde adres/commando, andere toggle):
  - `[RC5] Adres: 0x__ | Commando: 0x__ | Toggle: 0`
  - `[RC5] Adres: 0x__ | Commando: 0x__ | Toggle: 1`

**Nut van de toggle bit:**
- Onderscheid maken tussen “knop blijft ingedrukt / repeats” en “nieuwe toetsdruk”.
- In toepassingen (zoals LaserTag) kan je hiermee bv. één schot per druk detecteren, en herhalingen negeren of apart behandelen.

---

## 7. Problemen en oplossingen (verplicht)

- **PuTTY (VCP UART) werkt niet volledig / geen of onleesbare output** → soms kwam er geen tekst door, of leek de output “vast te hangen”. Oorzaken die ik heb nagekeken: juiste COM-poort, 115200 8N1, juiste TX/RX-pinnen (PA2/PA15), en of `printf()` wel naar USART2 werd doorgestuurd. Opgelost door PuTTY correct in te stellen en `printf()` om te leiden via `__io_putchar()`/`_write()`, met buffering uit (`setvbuf(stdout, NULL, _IONBF, 0)`).

- **Interferentie door andere zenders (klasgenoten)** → wanneer anderen in de buurt ook met IR bezig zijn, kan de TSOP4838 ook hun frames oppikken. Dit verstoort zowel de decodering als de oscilloscoopmetingen (bv. extra pulsen of “verkeerde” frames waardoor Single-trigger niet het juiste pakket capteert). Opgelost door tijdens scope-metingen zo veel mogelijk één zender tegelijk te gebruiken, dichter bij de ontvanger te meten, de TSOP4838 af te schermen, en de scope in *Single* met correcte trigger op de eerste falling edge van het gewenste frame te zetten.



---

## 8. Conclusie
De IR-receiver met TSOP4838 en STM32L432KC kon RC5-frames ontvangen en decoderen naar **adres**, **commando** en **toggle bit**. Tijdens de opbouw waren er twee praktische aandachtspunten: (1) de UART-debug via PuTTY werkte initieel niet betrouwbaar (geen/rare output), wat opgelost werd door de correcte VCP-instellingen en het omleiden van `printf()` naar USART2, en (2) in een klasomgeving werd soms ook IR van andere groepen ontvangen, wat zowel de decoding als de oscilloscoopbeelden kon verstoren. Door tijdens metingen interferentie te beperken (afstand/richting/afschermen en *Single* + juiste trigger) werden consistente scope-captures mogelijk.

De oscilloscoopmetingen tonen bitperioden in dezelfde grootteorde als de RC5-theorie, met een beperkte afwijking zoals weergegeven in de meet-tabel. Tot slot wisselt de toggle bit zoals verwacht, waardoor herhaalde frames van een ingedrukte knop te onderscheiden zijn van een nieuwe toetsdruk.



