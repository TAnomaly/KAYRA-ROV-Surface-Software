# KAYRA ROV — Raspberry Pi 5 Pin Baglanti Semasi

---

## Raspberry Pi 5 GPIO Header (40-pin)

```
                    ┌─────────────────────┐
              3.3V  │  1 ●  ○  2 │  5V
  ► SDA (I2C) GP2  │  3 ●  ○  4 │  5V
  ► SCL (I2C) GP3  │  5 ●  ○  6 │  GND ◄── PCA9685 GND
              GP4  │  7 ○  ○  8 │  GP14
              GND  │  9 ○  ○ 10 │  GP15
              GP17 │ 11 ○  ○ 12 │  GP18
              GP27 │ 13 ○  ○ 14 │  GND
              GP22 │ 15 ○  ○ 16 │  GP23
              3.3V │ 17 ○  ○ 18 │  GP24
              GP10 │ 19 ○  ○ 20 │  GND
              GP9  │ 21 ○  ○ 22 │  GP25
              GP11 │ 23 ○  ○ 24 │  GP8
              GND  │ 25 ○  ○ 26 │  GP7
              GP0  │ 27 ○  ○ 28 │  GP1
              GP5  │ 29 ○  ○ 30 │  GND
              GP6  │ 31 ○  ○ 32 │  GP12
              GP13 │ 33 ○  ○ 34 │  GND
              GP19 │ 35 ○  ○ 36 │  GP16
              GP26 │ 37 ○  ○ 38 │  GP20
              GND  │ 39 ○  ○ 40 │  GP21
                    └─────────────────────┘
                          ● = kullanilan pin
```

---

## PCA9685 → RPi 5 Baglantisi (I2C)

| PCA9685 Pin | RPi Pin | RPi GPIO | Aciklama |
|-------------|---------|----------|----------|
| **VCC**     | Pin 1   | 3.3V     | PCA9685 lojik besleme |
| **GND**     | Pin 6   | GND      | Ortak toprak |
| **SDA**     | Pin 3   | GPIO 2   | I2C veri hatti |
| **SCL**     | Pin 5   | GPIO 3   | I2C saat hatti |
| **V+**      | —       | —        | **HARICI** ESC guc kaynagi (5-6V) |

```
RPi 5                          PCA9685
┌──────┐                      ┌──────────────┐
│ 3.3V ├──── Pin 1 ──────────►│ VCC          │
│ GND  ├──── Pin 6 ──────────►│ GND          │
│ SDA  ├──── Pin 3 (GP2) ────►│ SDA          │
│ SCL  ├──── Pin 5 (GP3) ────►│ SCL          │
└──────┘                      │              │
                              │ V+  ◄────────── ESC Guc Kaynagi (5-6V)
                              │ GND ◄────────── ESC Guc GND (ortak)
                              └──────────────┘
```

> **UYARI:** PCA9685'in V+ pini RPi'den beslenmez! ESC'lerin guc
> kaynagindan (batarya regülatoru / BEC) baglanir. RPi sadece
> VCC (3.3V) ile lojik tarafi besler.

---

## PCA9685 → ESC Baglantisi (6 Motor)

| PCA9685 Kanal | Motor | Pozisyon | ESC Kablo Rengi |
|---------------|-------|----------|-----------------|
| **CH0**       | FR    | On-Sag yatay | Sinyal (beyaz/sari) |
| **CH1**       | FL    | On-Sol yatay | Sinyal (beyaz/sari) |
| **CH2**       | BR    | Arka-Sag yatay | Sinyal (beyaz/sari) |
| **CH3**       | BL    | Arka-Sol yatay | Sinyal (beyaz/sari) |
| **CH4**       | VL    | Dikey Sol | Sinyal (beyaz/sari) |
| **CH5**       | VR    | Dikey Sag | Sinyal (beyaz/sari) |

```
PCA9685                          ESC'ler
┌──────────┐
│ CH0  SIG ├─────────────────► FR ESC sinyal (beyaz)
│ CH0  GND ├─────────────────► FR ESC GND    (siyah)
│          │
│ CH1  SIG ├─────────────────► FL ESC sinyal
│ CH1  GND ├─────────────────► FL ESC GND
│          │
│ CH2  SIG ├─────────────────► BR ESC sinyal
│ CH2  GND ├─────────────────► BR ESC sinyal
│          │
│ CH3  SIG ├─────────────────► BL ESC sinyal
│ CH3  GND ├─────────────────► BL ESC GND
│          │
│ CH4  SIG ├─────────────────► VL ESC sinyal
│ CH4  GND ├─────────────────► VL ESC GND
│          │
│ CH5  SIG ├─────────────────► VR ESC sinyal
│ CH5  GND ├─────────────────► VR ESC GND
└──────────┘
```

> Her ESC cikisinda 3 kablo vardir:
> - **Sinyal** (beyaz/sari) → PCA9685 kanal SIG pinine
> - **GND** (siyah/kahve) → PCA9685 kanal GND pinine
> - **Power** (kirmizi) → **BAGLANMAZ** (ESC kendi gucunu bataryadan alir)

---

## Motor Pozisyonlari (Ustten Gorunum)

```
              ON (ileri)
                ▲
                │
        FL ╲    │    ╱ FR          CH1, CH0
     (CH1)  ╲   │   ╱  (CH0)
              ╲  │  ╱
               ╲ │ ╱
          ┌─────────────┐
  SOL ◄── │             │ ──► SAG
          │   KAYRA     │
          │    ROV      │
          └─────────────┘
               ╱ │ ╲
              ╱  │  ╲
     (CH3)  ╱   │   ╲  (CH2)
        BL ╱    │    ╲ BR          CH3, CH2
                │
                ▼
              ARKA

      Dikey motorlar (ust/alt):
        VL (CH4) — sol taraf
        VR (CH5) — sag taraf
```

---

## ArduCam → RPi 5

| ArduCam | RPi | Aciklama |
|---------|-----|----------|
| CSI ribbon kablo | CSI konnektoru | RPi 5 uzerindeki kamera portu |

> RPi 5'te **2 adet CSI konnektoru** vardir.
> ArduCam'i bunlardan birine baglayin (genellikle CAM0).

---

## Phantom Sublink → RPi 5

| Sublink ROV Board | RPi | Aciklama |
|-------------------|-----|----------|
| Ethernet | Ethernet portu | RJ45 kablo ile dogrudan |

> RPi 5 statik IP: `192.168.2.2`
>
> Ayarlamak icin:
> ```bash
> sudo nmcli con mod "Wired connection 1" \
>   ipv4.addresses 192.168.2.2/24 \
>   ipv4.method manual
> sudo nmcli con up "Wired connection 1"
> ```

---

## PWM Degerleri

| Deger | PWM (µs) | Anlami |
|-------|----------|--------|
| Minimum | 1100 µs | Tam ters |
| Notr | 1500 µs | Durma |
| Maksimum | 1900 µs | Tam ileri |

> ESC'ler baslatildiginda 2 saniye boyunca 1500 µs (notr) sinyal
> gonderilir — bu ESC arm islemidir.

---

## Tam Kablolama Ozeti

```
[Batarya] ──► [Guc Dagitim] ──┬──► ESC x6 ──► Motor x6
                               │
                               └──► PCA9685 V+ (servo guc)

[RPi 5]
  ├── Pin 1 (3.3V) ────► PCA9685 VCC
  ├── Pin 3 (SDA)  ────► PCA9685 SDA
  ├── Pin 5 (SCL)  ────► PCA9685 SCL
  ├── Pin 6 (GND)  ────► PCA9685 GND ──► ESC GND (ortak)
  ├── CSI port     ────► ArduCam
  └── Ethernet     ────► Sublink ROV board ══ tether ══► Sublink Topside ──► PC
```
