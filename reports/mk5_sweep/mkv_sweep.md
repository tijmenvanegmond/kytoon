# Mk V «Manta» — design sweep

Baseline: chord 13.3 m, taper 0.35, 5 cells, 0.1 bar — v_max 23.1 m/s (tether WLL), net lift incl. tether 232 kg, L1 cr_ratio 1.01

## center chord [m] (fat_wing.chord)

| value | v_min | v_max | limiter | tow@12 [kN] | net lift incl tether [kg] | hoop util | bend util | structure OK | L1 cr_ratio | L1 flags |
|---|---|---|---|---|---|---|---|---|---|---|
| 11.5 | 0.0 | 24.9 | tether WLL | 15.5 | 113 | 10% | 14% | ✔ | 1.01 | fat section t/c=0.28 is beyond the Breukels LEI regression fit range — section polar extrapolated; twin-skin section approximated by slim Breukels LEI profile (t=0.06) — expect conservative L/D |
| 13.3 (baseline) | 0.0 | 23.1 | tether WLL | 18.0 | 232 | 11% | 10% | ✔ | 1.01 | fat section t/c=0.28 is beyond the Breukels LEI regression fit range — section polar extrapolated; twin-skin section approximated by slim Breukels LEI profile (t=0.06) — expect conservative L/D |
| 15 | 0.0 | 21.8 | tether WLL | 20.2 | 363 | 12% | 8% | ✔ | 1.01 | fat section t/c=0.28 is beyond the Breukels LEI regression fit range — section polar extrapolated; twin-skin section approximated by slim Breukels LEI profile (t=0.06) — expect conservative L/D |

## taper λ (fat_wing.taper)

| value | v_min | v_max | limiter | tow@12 [kN] | net lift incl tether [kg] | hoop util | bend util | structure OK | L1 cr_ratio | L1 flags |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.25 | 0.0 | 24.1 | tether WLL | 16.6 | 183 | 11% | 9% | ✔ | 1.01 | fat section t/c=0.28 is beyond the Breukels LEI regression fit range — section polar extrapolated; twin-skin section approximated by slim Breukels LEI profile (t=0.06) — expect conservative L/D |
| 0.35 (baseline) | 0.0 | 23.1 | tether WLL | 18.0 | 232 | 11% | 10% | ✔ | 1.01 | fat section t/c=0.28 is beyond the Breukels LEI regression fit range — section polar extrapolated; twin-skin section approximated by slim Breukels LEI profile (t=0.06) — expect conservative L/D |
| 0.45 | 0.0 | 22.3 | tether WLL | 19.2 | 288 | 11% | 11% | ✔ | 1.01 | fat section t/c=0.28 is beyond the Breukels LEI regression fit range — section polar extrapolated; twin-skin section approximated by slim Breukels LEI profile (t=0.06) — expect conservative L/D |

## cells (fat_wing.n_cells)

| value | v_min | v_max | limiter | tow@12 [kN] | net lift incl tether [kg] | hoop util | bend util | structure OK | L1 cr_ratio | L1 flags |
|---|---|---|---|---|---|---|---|---|---|---|
| 3 | 0.0 | 23.1 | tether WLL | 18.0 | 232 | 18% | 10% | ✔ | 1.01 | fat section t/c=0.28 is beyond the Breukels LEI regression fit range — section polar extrapolated; twin-skin section approximated by slim Breukels LEI profile (t=0.06) — expect conservative L/D |
| 5 (baseline) | 0.0 | 23.1 | tether WLL | 18.0 | 232 | 11% | 10% | ✔ | 1.01 | fat section t/c=0.28 is beyond the Breukels LEI regression fit range — section polar extrapolated; twin-skin section approximated by slim Breukels LEI profile (t=0.06) — expect conservative L/D |
| 7 | 0.0 | 23.1 | tether WLL | 18.0 | 232 | 8% | 10% | ✔ | 1.01 | fat section t/c=0.28 is beyond the Breukels LEI regression fit range — section polar extrapolated; twin-skin section approximated by slim Breukels LEI profile (t=0.06) — expect conservative L/D |

## cell pressure [bar] (fat_wing.pressure_bar)

| value | v_min | v_max | limiter | tow@12 [kN] | net lift incl tether [kg] | hoop util | bend util | structure OK | L1 cr_ratio | L1 flags |
|---|---|---|---|---|---|---|---|---|---|---|
| 0.07 | 0.0 | 23.1 | tether WLL | 18.0 | 232 | 8% | 15% | ✔ | 1.01 | fat section t/c=0.28 is beyond the Breukels LEI regression fit range — section polar extrapolated; twin-skin section approximated by slim Breukels LEI profile (t=0.06) — expect conservative L/D |
| 0.1 (baseline) | 0.0 | 23.1 | tether WLL | 18.0 | 232 | 11% | 10% | ✔ | 1.01 | fat section t/c=0.28 is beyond the Breukels LEI regression fit range — section polar extrapolated; twin-skin section approximated by slim Breukels LEI profile (t=0.06) — expect conservative L/D |
| 0.14 | 0.0 | 23.1 | tether WLL | 18.0 | 232 | 16% | 7% | ✔ | 1.01 | fat section t/c=0.28 is beyond the Breukels LEI regression fit range — section polar extrapolated; twin-skin section approximated by slim Breukels LEI profile (t=0.06) — expect conservative L/D |
