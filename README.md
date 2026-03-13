# Noor
This is repository for Noor the educational toy

# Pin Configuration (Initial Draft)

This section lists the **initial GPIO assignments** for the ESP32‑S3 N16R8 module used in this project.  
Please note: these mappings are **provisional** and may be updated based on hardware validation and testing results.

---

## I2S Audio (MAX98357 Amplifier)

| Signal      | GPIO    | Notes                                   |
|-------------|---------|-----------------------------------------|
| BCLK        | GPIO18  | Bit clock                               |
| LRCLK / WS  | GPIO17  | Word select (left/right clock)          |
| DIN         | GPIO16  | Data input                              |
| GAIN        | GPIO9   | Configures amplifier gain(not used yet) |


Connections Test Successful
---

## SD Card (SPI Mode)

| Signal | GPIO   | Notes                        |
|--------|--------|------------------------------|
| CS     | GPIO10 | Chip select                  |
| MOSI   | GPIO11 | Master out, slave in         |
| SCK    | GPIO12 | Clock                        |
| MISO   | GPIO13 | Master in, slave out         |

Connections Test Successful
---

## Rotary Encoders

| Encoder   | Signal | GPIO   |
|-----------|--------|--------|
| Encoder 1 | CLK    | GPIO1  |
| Encoder 1 | DT     | GPIO2  |
| Encoder 1 | SW     | GPIO21 |

*(Additional encoders can be mapped similarly in future revisions.)*

Connections Test Successful
---

## Pushbuttons

| Function     | GPIO   |
|--------------|--------|
| Play / Pause | GPIO14 |
| Back / Home  | GPIO15 |
| Volume +     | GPIO4  |
| Volume –     | GPIO5  |

Connections Test Successful
---

## Audio Jack

| Connection | Pin Description |
|------------|-----------------|
| Speaker + | SW (switch second pin from left when jack face is away) |
| Speaker - | MAX OUT - |
| MAX OUT + | TIP (most left pin from back when jack face is away) |
| GND | SLEEVE of audio jack (single pin of audio jack) |
| GPIO 48 | jack switch to detect headphones|

Connections Test Successful
---
## Reserved Pins (Do Not Use)

| Pin Group     | GPIOs                          | Reason                                  |
|---------------|--------------------------------|-----------------------------------------|
| Strapping     | GPIO0, GPIO3, GPIO45, GPIO46   | Affect boot mode and startup config      |
| PSRAM (Octal) | GPIO35, GPIO36, GPIO37         | Connected to embedded PSRAM              |
| Debug UART    | GPIO43, GPIO44                 | Default UART for debugging               |
| USB           | GPIO19, GPIO20                 | Dedicated to USB‑OTG functionality       |
| JTAG          | GPIO39, GPIO40, GPIO41, GPIO42 | Reserved for JTAG debugging            |
| RGB LED       | GPIO47                         | On‑module RGB LED                        |

---

## Notes

- All button and encoder inputs are configured for **interrupt handling** with internal pull‑ups enabled.  
- I2S and SPI pins are routed through the ESP32‑S3 GPIO matrix and can be reassigned if conflicts arise.  
- Reserved pins listed above must not be used for general I/O.  
- These assignments are **initial** and may change during integration and testing. Always refer to the latest revision of this README for updates.




#  SD Card Structure 

Simple guide to organize your SD card for the Noor Audio Player.

---

##  Quick Overview

 SD card has **two types of files**:

1. ** Announcement files** (in root) - Tell user what they're selecting
2. ** Story folders** - Contain the actual audio content

---

##  Complete File Structure

```
/sdcard/
│
├── announcements/          (FAT32 may show as ANNOUN~1)
│   │
│   ├── welcome.wav          ← boot greeting
│   ├── rotate.wav           ← "rotate to navigate"
│   ├── stories.wav          ← STORIES folder name
│   │
│   ├── ADAM.WAV             ← prophet folder names
│   ├── DAOUD.WAV
│   ├── IBRAHIM.WAV
│   ├── ISMAEEL.WAV
│   ├── ISSA.WAV
│   ├── MOUSSA.WAV
│   ├── MUHAMMAD.WAV
│   ├── NUH.WAV
│   ├── YAQOOB.WAV
│   ├── YOUSUF.WAV
│   │
│   ├── INSMUH.WAV           ← "entering Muhammad" announcement
│   │
│   ├── A1ANN.WAV            ← Adam story track announcements
│   ├── A2ANN.WAV
│   ├── A3ANN.WAV
│   ├── A4ANN.WAV
│   ├── A5ANN.WAV
│   ├── A6ANN.WAV
│   ├── A7ANN.WAV
│   ├── A8ANN.WAV
│   ├── A9ANN.WAV
│   ├── A10ANN.WAV
│   │
│   ├── D1ANN.WAV            ← Daoud story track announcements
│   ├── D2ANN.WAV
│   ├── D3ANN.WAV
│   ├── D4ANN.WAV
│   ├── D5ANN.WAV
│   ├── D6ANN.WAV
│   ├── D7ANN.WAV
│   ├── D8ANN.WAV
│   ├── D9ANN.WAV
│   ├── D10ANN.WAV
│   │
│   ├── IB1ANN.WAV           ← Ibrahim story track announcements
│   ├── IB2ANN.WAV
│   ├── IB3ANN.WAV
│   ├── IB4ANN.WAV
│   ├── IB5ANN.WAV
│   ├── IB6ANN.WAV
│   ├── IB7ANN.WAV
│   ├── IB8ANN.WAV
│   ├── IB9ANN.WAV
│   ├── IB10ANN.WAV
│   │
│   ├── IS1ANN.WAV           ← Ismaeel story track announcements
│   ├── IS2ANN.WAV
│   ├── IS3ANN.WAV
│   ├── IS4ANN.WAV
│   ├── IS5ANN.WAV
│   ├── IS6ANN.WAV
│   ├── IS7ANN.WAV
│   ├── IS8ANN.WAV
│   ├── IS9ANN.WAV
│   ├── IS10ANN.WAV
│   │
│   ├── I1ANN.WAV            ← Issa story track announcements
│   ├── I2ANN.WAV
│   ├── I3ANN.WAV
│   ├── I4ANN.WAV
│   ├── I5ANN.WAV
│   ├── I6ANN.WAV
│   ├── I7ANN.WAV
│   ├── I8ANN.WAV
│   ├── I9ANN.WAV
│   ├── I10ANN.WAV
│   │
│   ├── MS1ANN.WAV           ← Moussa story track announcements
│   ├── MS2ANN.WAV
│   ├── MS3ANN.WAV
│   ├── MS4ANN.WAV
│   ├── MS5ANN.WAV
│   ├── MS6ANN.WAV
│   ├── MS7ANN.WAV
│   ├── MS8ANN.WAV
│   ├── MS9ANN.WAV
│   ├── MS10ANN.WAV
│   │
│   ├── M1ANN.WAV            ← Muhammad story track announcements
│   ├── M2ANN.WAV
│   ├── M3ANN.WAV
│   ├── M4ANN.WAV
│   ├── M5ANN.WAV
│   ├── M6ANN.WAV
│   ├── M7ANN.WAV
│   ├── M8ANN.WAV
│   ├── M9ANN.WAV
│   ├── M10ANN.WAV
│   │
│   ├── N1ANN.WAV            ← Nuh story track announcements
│   ├── N2ANN.WAV
│   ├── N3ANN.WAV
│   ├── N4ANN.WAV
│   ├── N5ANN.WAV
│   ├── N6ANN.WAV
│   ├── N7ANN.WAV
│   ├── N8ANN.WAV
│   ├── N9ANN.WAV
│   ├── N10ANN.WAV
│   │
│   ├── YQ1ANN.WAV           ← Yaqoob story track announcements
│   ├── YQ2ANN.WAV
│   ├── YQ3ANN.WAV
│   ├── YQ4ANN.WAV
│   ├── YQ5ANN.WAV
│   ├── YQ6ANN.WAV
│   ├── YQ7ANN.WAV
│   ├── YQ8ANN.WAV
│   ├── YQ9ANN.WAV
│   ├── YQ10ANN.WAV
│   │
│   ├── Y1ANN.WAV            ← Yousuf story track announcements
│   ├── Y2ANN.WAV
│   ├── Y3ANN.WAV
│   ├── Y4ANN.WAV
│   ├── Y5ANN.WAV
│   ├── Y6ANN.WAV
│   ├── Y7ANN.WAV
│   ├── Y8ANN.WAV
│   ├── Y9ANN.WAV
│   ├── Y10ANN.WAV
│   │
│   ├── v1.wav               ← volume level announcements
│   ├── v2.wav
│   ├── v3.wav
│   ├── v4.wav
│   ├── v5.wav
│   ├── v6.wav
│   ├── v7.wav
│   ├── v8.wav
│   ├── v9.wav
│   ├── v10.wav
│   │
│   ├── correct.wav          ← quiz correct answer sound
│   ├── wrong.wav            ← quiz wrong answer sound
│   ├── b1.wav               ← battery alerts
│   ├── b2.wav
│   ├── b3.wav
│   │
│   ├── TA.wav               ← TAWID folder name announcement
│   ├── TT1.wav              ← TAWID track announcements
│   ├── TT2.wav
│   ├── TT3.wav
│   ├── TT4.wav
│   ├── TT5.wav
│   ├── TT6.wav
│   ├── TT7.wav
│   ├── TT8.wav
│   ├── TT9.wav
│   ├── TT10.wav
│   ├── TT11.wav
│   ├── TT12.wav
│   ├── TT13.wav
│   ├── TT14.wav
│   ├── TT15.wav
│   ├── TT16.wav
│   ├── TT17.wav
│   ├── TT18.wav
│   ├── TT19.wav
│   └── TT20.wav
│
├── STORIES/
│   │
│   ├── ADAM/
│   │   ├── a1.wav           ← story files (shown in navigation)
│   │   ├── a2.wav
│   │   ├── a3.wav
│   │   ├── a4.wav
│   │   ├── a5.wav
│   │   ├── a6.wav
│   │   ├── a7.wav
│   │   ├── a8.wav
│   │   ├── a9.wav
│   │   ├── a10.wav
│   │   ├── sa1_1.wav   crra1_1.wav   wra1_1.wav    ← story 1, quiz 1
│   │   ├── sa1_2.wav   crra1_2.wav   wra1_2.wav    ← story 1, quiz 2
│   │   ├── sa1_3.wav   crra1_3.wav   wra1_3.wav    ← story 1, quiz 3
│   │   ├── sa2_1.wav   crra2_1.wav   wra2_1.wav    ← story 2, quiz 1
│   │   ├── sa2_2.wav   crra2_2.wav   wra2_2.wav
│   │   ├── sa2_3.wav   crra2_3.wav   wra2_3.wav
│   │   ├── sa3_1.wav   crra3_1.wav   wra3_1.wav
│   │   ├── sa3_2.wav   crra3_2.wav   wra3_2.wav
│   │   ├── sa3_3.wav   crra3_3.wav   wra3_3.wav
│   │   ├── sa4_1.wav   crra4_1.wav   wra4_1.wav
│   │   ├── sa4_2.wav   crra4_2.wav   wra4_2.wav
│   │   ├── sa4_3.wav   crra4_3.wav   wra4_3.wav
│   │   ├── sa5_1.wav   crra5_1.wav   wra5_1.wav
│   │   ├── sa5_2.wav   crra5_2.wav   wra5_2.wav
│   │   ├── sa5_3.wav   crra5_3.wav   wra5_3.wav
│   │   ├── sa6_1.wav   crra6_1.wav   wra6_1.wav
│   │   ├── sa6_2.wav   crra6_2.wav   wra6_2.wav
│   │   ├── sa6_3.wav   crra6_3.wav   wra6_3.wav
│   │   ├── sa7_1.wav   crra7_1.wav   wra7_1.wav
│   │   ├── sa7_2.wav   crra7_2.wav   wra7_2.wav
│   │   ├── sa7_3.wav   crra7_3.wav   wra7_3.wav
│   │   ├── sa8_1.wav   crra8_1.wav   wra8_1.wav
│   │   ├── sa8_2.wav   crra8_2.wav   wra8_2.wav
│   │   ├── sa8_3.wav   crra8_3.wav   wra8_3.wav
│   │   ├── sa9_1.wav   crra9_1.wav   wra9_1.wav
│   │   ├── sa9_2.wav   crra9_2.wav   wra9_2.wav
│   │   ├── sa9_3.wav   crra9_3.wav   wra9_3.wav
│   │   ├── sa10_1.wav  crra10_1.wav  wra10_1.wav
│   │   ├── sa10_2.wav  crra10_2.wav  wra10_2.wav
│   │   └── sa10_3.wav  crra10_3.wav  wra10_3.wav
│   │
│   ├── DAOUD/
│   │   ├── d1.wav … d10.wav
│   │   ├── sd1_1.wav   crrd1_1.wav   wrd1_1.wav
│   │   ├── sd1_2.wav   crrd1_2.wav   wrd1_2.wav
│   │   ├── sd1_3.wav   crrd1_3.wav   wrd1_3.wav
│   │   ├── sd2_1.wav   crrd2_1.wav   wrd2_1.wav
│   │   ├── … (same pattern for stories 2–9)
│   │   ├── sd10_1.wav  crrd10_1.wav  wrd10_1.wav
│   │   ├── sd10_2.wav  crrd10_2.wav  wrd10_2.wav
│   │   └── sd10_3.wav  crrd10_3.wav  wrd10_3.wav
│   │
│   ├── IBRAHIM/
│   │   ├── ib1.wav … ib10.wav
│   │   ├── sib1_1.wav   crrib1_1.wav   wrib1_1.wav
│   │   ├── sib1_2.wav   crrib1_2.wav   wrib1_2.wav
│   │   ├── sib1_3.wav   crrib1_3.wav   wrib1_3.wav
│   │   ├── … (same pattern for stories 2–9)
│   │   ├── sib10_1.wav  crrib10_1.wav  wrib10_1.wav
│   │   ├── sib10_2.wav  crrib10_2.wav  wrib10_2.wav
│   │   └── sib10_3.wav  crrib10_3.wav  wrib10_3.wav
│   │
│   ├── ISMAEEL/
│   │   ├── is1.wav … is10.wav
│   │   ├── sis1_1.wav   crris1_1.wav   wris1_1.wav
│   │   ├── sis1_2.wav   crris1_2.wav   wris1_2.wav
│   │   ├── sis1_3.wav   crris1_3.wav   wris1_3.wav
│   │   ├── … (same pattern for stories 2–9)
│   │   ├── sis10_1.wav  crris10_1.wav  wris10_1.wav
│   │   ├── sis10_2.wav  crris10_2.wav  wris10_2.wav
│   │   └── sis10_3.wav  crris10_3.wav  wris10_3.wav
│   │
│   ├── ISSA/
│   │   ├── i1.wav … i10.wav
│   │   ├── si1_1.wav   crri1_1.wav   wri1_1.wav
│   │   ├── si1_2.wav   crri1_2.wav   wri1_2.wav
│   │   ├── si1_3.wav   crri1_3.wav   wri1_3.wav
│   │   ├── … (same pattern for stories 2–9)
│   │   ├── si10_1.wav  crri10_1.wav  wri10_1.wav
│   │   ├── si10_2.wav  crri10_2.wav  wri10_2.wav
│   │   └── si10_3.wav  crri10_3.wav  wri10_3.wav
│   │
│   ├── MOUSSA/
│   │   ├── ms1.wav … ms10.wav
│   │   ├── sms1_1.wav   crrms1_1.wav   wrms1_1.wav
│   │   ├── sms1_2.wav   crrms1_2.wav   wrms1_2.wav
│   │   ├── sms1_3.wav   crrms1_3.wav   wrms1_3.wav
│   │   ├── … (same pattern for stories 2–9)
│   │   ├── sms10_1.wav  crrms10_1.wav  wrms10_1.wav
│   │   ├── sms10_2.wav  crrms10_2.wav  wrms10_2.wav
│   │   └── sms10_3.wav  crrms10_3.wav  wrms10_3.wav
│   │
│   ├── MUHAMMAD/
│   │   ├── m1.wav … m10.wav
│   │   ├── sm1_1.wav   crrm1_1.wav   wrm1_1.wav
│   │   ├── sm1_2.wav   crrm1_2.wav   wrm1_2.wav
│   │   ├── sm1_3.wav   crrm1_3.wav   wrm1_3.wav
│   │   ├── … (same pattern for stories 2–9)
│   │   ├── sm10_1.wav  crrm10_1.wav  wrm10_1.wav
│   │   ├── sm10_2.wav  crrm10_2.wav  wrm10_2.wav
│   │   └── sm10_3.wav  crrm10_3.wav  wrm10_3.wav
│   │
│   ├── NUH/
│   │   ├── n1.wav … n10.wav
│   │   ├── sn1_1.wav   crrn1_1.wav   wrn1_1.wav
│   │   ├── sn1_2.wav   crrn1_2.wav   wrn1_2.wav
│   │   ├── sn1_3.wav   crrn1_3.wav   wrn1_3.wav
│   │   ├── … (same pattern for stories 2–9)
│   │   ├── sn10_1.wav  crrn10_1.wav  wrn10_1.wav
│   │   ├── sn10_2.wav  crrn10_2.wav  wrn10_2.wav
│   │   └── sn10_3.wav  crrn10_3.wav  wrn10_3.wav
│   │
│   ├── YAQOOB/
│   │   ├── yq1.wav … yq10.wav
│   │   ├── syq1_1.wav   crryq1_1.wav   wryq1_1.wav
│   │   ├── syq1_2.wav   crryq1_2.wav   wryq1_2.wav
│   │   ├── syq1_3.wav   crryq1_3.wav   wryq1_3.wav
│   │   ├── … (same pattern for stories 2–9)
│   │   ├── syq10_1.wav  crryq10_1.wav  wryq10_1.wav
│   │   ├── syq10_2.wav  crryq10_2.wav  wryq10_2.wav
│   │   └── syq10_3.wav  crryq10_3.wav  wryq10_3.wav
│   │
│   └── YOUSUF/
│       ├── y1.wav … y10.wav
│       ├── sy1_1.wav   crry1_1.wav   wry1_1.wav
│       ├── sy1_2.wav   crry1_2.wav   wry1_2.wav
│       ├── sy1_3.wav   crry1_3.wav   wry1_3.wav
│       ├── … (same pattern for stories 2–9)
│       ├── sy10_1.wav  crry10_1.wav  wry10_1.wav
│       ├── sy10_2.wav  crry10_2.wav  wry10_2.wav
│       └── sy10_3.wav  crry10_3.wav  wry10_3.wav
│
├── TAWID/
│   ├── TT1.wav
│   ├── TT2.wav
│   ├── TT3.wav
│   ├── TT4.wav
│   ├── TT5.wav
│   ├── TT6.wav
│   ├── TT7.wav
│   ├── TT8.wav
│   ├── TT9.wav
│   ├── TT10.wav
│   ├── TT11.wav
│   ├── TT12.wav
│   ├── TT13.wav
│   ├── TT14.wav
│   ├── TT15.wav
│   ├── TT16.wav
│   ├── TT17.wav
│   ├── TT18.wav
│   ├── TT19.wav
│   └── TT20.wav
│
└── update.bin              (optional — OTA firmware update)

```

---



##  File Naming Rules

###  **DO:**
- Use **no spaces**: `StoriesofProphets` 
- Use **no underscores**: `HazratMuhammad`  
- Keep names **short and clear**

###  **DON'T:**
- Use spaces: `Stories of Prophets` 
- Use underscores: `Stories_of_Prophets` 

---

##  Audio Requirements

| Property | Value |
|----------|-------|
| Format | WAV |
---


##  Adding New Content

### **Add New Prophet:**

**Step 1:** Create announcement
```
Yousuf.wav  ← Record "Prophet Yusuf"
```

**Step 2:** Create folder
```
STORIES/YOUSUF/
```

**Step 3:** Add stories
```
Tousuf/
├── y1.wav
├── y2.wav
└── y3.wav

Simmilarly for other prophets
```

### **Add More Tawid Lessons:**

**Step 1:** Create announcement
```
TT4.wav  ← Record "Lesson Four"
```

**Step 2:** Add story file
```
TAWID/t4.wav
```


# Instructions

https://docs.google.com/document/d/1RYAxxgP-j9_PKx_9E5XExqXV5vHtyEG8tLyEzbizOWY/edit?tab=t.0
