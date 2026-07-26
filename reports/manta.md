# Manta Type Comparison Report

L0 analytic design layer for Manta Type A-Z variants.
Closed-form physics: Archimedes buoyancy, pressurized-beam wrinkle margins,
quasi-static wind envelope. All numbers ISA sea level.

## Comparison

| Parameter | Manta Type A «Scout» | Manta Type B «Standard» | Manta Type C «Heavy Lift» | Manta Type D «Long Range» | Manta Type E «High Altitude» |
|---|---|---|---|---|---|
| Type | A | B | C | D | E |
| Name | Manta Type A «Scout» | Manta Type B «Standard» | Manta Type C «Heavy Lift» | Manta Type D «Long Range» | Manta Type E «High Altitude» |
| Wing area [m²] | 120 | 288 | 360 | 364 | 330 |
| He volume [m³] | 143 | 530 | 785 | 693 | 685 |
| Flying mass [kg] | 111 | 271 | 502 | 327 | 304 |
| Gross He lift [kg] | 150 | 554 | 821 | 725 | 717 |
| Net static lift [kg] | 39 | 284 | 319 | 398 | 413 |
| Net incl. tether [kg] | 12 | 228 | 209 | 254 | 273 |
| Calm-air capable | ✔ | ✔ | ✔ | ✔ | ✔ |
| v_min [m/s] | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| v_max [m/s] | 22.7 | 23.1 | 25.0 | 21.8 | 21.7 |
| v_max limiter | tether WLL | tether WLL | tether WLL | tether WLL | tether WLL |
| Tow @12 m/s [kN] | 8.0 | 18.0 | 23.1 | 25.2 | 19.2 |
| Spare vert. lift @10 m/s [kg] | 442 | 1,192 | 1,412 | 1,651 | 1,371 |
| Payload [kg] | 20 | 60 | 200 | 80 | 40 |
| Tether length [m] | 300 | 400 | 500 | 800 | 1,000 |

## Structure Margins


### Manta Type A «Scout» — structure @ tow 8.0 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø2.4 m) | 10.6 | 10.6% | 2.45 | 26.47 | 9.2% | ✔ |


### Manta Type B «Standard» — structure @ tow 18.0 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø3.7 m) | 13.3 | 11.1% | 10.37 | 101.40 | 10.2% | ✔ |


### Manta Type C «Heavy Lift» — structure @ tow 23.1 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø4.5 m) | 15.0 | 10.7% | 16.65 | 214.71 | 7.8% | ✔ |


### Manta Type D «Long Range» — structure @ tow 25.2 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø3.9 m) | 9.0 | 8.2% | 18.17 | 106.45 | 17.1% | ✔ |


### Manta Type E «High Altitude» — structure @ tow 19.2 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø4.2 m) | 15.9 | 12.3% | 11.76 | 160.59 | 7.3% | ✔ |

## Wind Envelopes


### Manta Type A «Scout» — wind envelope

- **Operating range**: 0.0 – 22.7 m/s
- **v_max limited by**: tether WLL
- **Tow force @ 12 m/s**: 8.0 kN
- **Vertical capacity @ 10 m/s**: 442.2 kg


### Manta Type B «Standard» — wind envelope

- **Operating range**: 0.0 – 23.1 m/s
- **v_max limited by**: tether WLL
- **Tow force @ 12 m/s**: 18.0 kN
- **Vertical capacity @ 10 m/s**: 1,192.1 kg


### Manta Type C «Heavy Lift» — wind envelope

- **Operating range**: 0.0 – 25.0 m/s
- **v_max limited by**: tether WLL
- **Tow force @ 12 m/s**: 23.1 kN
- **Vertical capacity @ 10 m/s**: 1,411.7 kg


### Manta Type D «Long Range» — wind envelope

- **Operating range**: 0.0 – 21.8 m/s
- **v_max limited by**: tether WLL
- **Tow force @ 12 m/s**: 25.2 kN
- **Vertical capacity @ 10 m/s**: 1,650.8 kg


### Manta Type E «High Altitude» — wind envelope

- **Operating range**: 0.0 – 21.7 m/s
- **v_max limited by**: tether WLL
- **Tow force @ 12 m/s**: 19.2 kN
- **Vertical capacity @ 10 m/s**: 1,370.6 kg

## Design Notes

- **Manta Type A «Scout»**: tether-limited top speed
- **Manta Type B «Standard»**: tether-limited top speed
- **Manta Type C «Heavy Lift»**: tether-limited top speed
- **Manta Type D «Long Range»**: tether-limited top speed
- **Manta Type E «High Altitude»**: tether-limited top speed