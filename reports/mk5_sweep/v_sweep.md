# Mk V «Manta» — design sweep

Baseline: chord 13.3 m, taper 0.35, 5 cells, 0.1 bar — v_max 23.1 m/s (tether WLL), net lift incl. tether 232 kg

## center chord [m] (fat_wing.chord)

| value | v_min | v_max | limiter | tow@12 [kN] | net lift incl tether [kg] | structure OK |
|---|---|---|---|---|---|---|
| 11.5 | 0.0 | 24.9 | tether WLL | 15.5 | 113 | ✔ |
| 13.3 (baseline) | 0.0 | 23.1 | tether WLL | 18.0 | 232 | ✔ |
| 15 | 0.0 | 21.8 | tether WLL | 20.2 | 363 | ✔ |

## taper λ (fat_wing.taper)

| value | v_min | v_max | limiter | tow@12 [kN] | net lift incl tether [kg] | structure OK |
|---|---|---|---|---|---|---|
| 0.25 | 0.0 | 24.1 | tether WLL | 16.6 | 183 | ✔ |
| 0.35 (baseline) | 0.0 | 23.1 | tether WLL | 18.0 | 232 | ✔ |
| 0.45 | 0.0 | 22.3 | tether WLL | 19.2 | 288 | ✔ |

## cells (fat_wing.n_cells)

| value | v_min | v_max | limiter | tow@12 [kN] | net lift incl tether [kg] | structure OK |
|---|---|---|---|---|---|---|
| 3 | 0.0 | 23.1 | tether WLL | 18.0 | 232 | ✔ |
| 5 (baseline) | 0.0 | 23.1 | tether WLL | 18.0 | 232 | ✔ |
| 7 | 0.0 | 23.1 | tether WLL | 18.0 | 232 | ✔ |

## cell pressure [bar] (fat_wing.pressure_bar)

| value | v_min | v_max | limiter | tow@12 [kN] | net lift incl tether [kg] | structure OK |
|---|---|---|---|---|---|---|
| 0.07 | 0.0 | 23.1 | tether WLL | 18.0 | 232 | ✔ |
| 0.1 (baseline) | 0.0 | 23.1 | tether WLL | 18.0 | 232 | ✔ |
| 0.14 | 0.0 | 23.1 | tether WLL | 18.0 | 232 | ✔ |
