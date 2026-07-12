# Kytoon L0 Analysis — Mk V–V

Closed-form layer: Archimedes buoyancy, pressurized-beam wrinkle margins, quasi-static wind envelope. All numbers ISA sea level; tether drag/sag deferred to L1 (MoorPy).

## Comparison

| Parameter | Mk V «Manta» | Mk V «Manta» [chord=11.5] | Mk V «Manta» [chord=15.0] | Mk V «Manta» [taper=0.25] | Mk V «Manta» [taper=0.45] | Mk V «Manta» [n_cells=3] | Mk V «Manta» [n_cells=7] | Mk V «Manta» [pressure_bar=0.07] | Mk V «Manta» [pressure_bar=0.14] |
|---|---|---|---|---|---|---|---|---|---|
| Wing area [m²] | 288 | 248 | 324 | 266 | 309 | 288 | 288 | 288 | 288 |
| He volume [m³] | 530 | 396 | 674 | 472 | 595 | 530 | 530 | 530 | 530 |
| Flying mass [kg] | 266 | 245 | 286 | 255 | 278 | 266 | 266 | 266 | 266 |
| Gross He lift [kg] | 554 | 414 | 705 | 494 | 622 | 554 | 554 | 554 | 554 |
| Net static lift [kg] | 288 | 169 | 419 | 239 | 344 | 288 | 288 | 288 | 288 |
| … incl. tether [kg] | 232 | 113 | 363 | 183 | 288 | 232 | 232 | 232 | 232 |
| Calm-air capable | ✔ | ✔ | ✔ | ✔ | ✔ | ✔ | ✔ | ✔ | ✔ |
| v_min [m/s] | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| v_max [m/s] | 23.1 | 24.9 | 21.8 | 24.1 | 22.3 | 23.1 | 23.1 | 23.1 | 23.1 |
| v_max limiter | tether WLL | tether WLL | tether WLL | tether WLL | tether WLL | tether WLL | tether WLL | tether WLL | tether WLL |
| Tow @12 m/s [kN] | 18.0 | 15.5 | 20.2 | 16.6 | 19.2 | 18.0 | 18.0 | 18.0 | 18.0 |
| Spare vert. lift @10 m/s [kg] | 1,196 | 945 | 1,448 | 1,074 | 1,321 | 1,196 | 1,196 | 1,196 | 1,196 |

## Structure margins

### Mk V «Manta» — structure @ tow 18.0 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø3.7 m) | 13.30 | 11.1% | 10.37 | 101.40 | 10.2% | ✔ |

### Mk V «Manta» [chord=11.5] — structure @ tow 15.5 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø3.2 m) | 11.50 | 9.6% | 8.95 | 65.55 | 13.7% | ✔ |

### Mk V «Manta» [chord=15.0] — structure @ tow 20.2 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø4.2 m) | 15.00 | 12.5% | 11.67 | 145.47 | 8.0% | ✔ |

### Mk V «Manta» [taper=0.25] — structure @ tow 16.6 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø3.7 m) | 13.30 | 11.1% | 9.58 | 101.40 | 9.4% | ✔ |

### Mk V «Manta» [taper=0.45] — structure @ tow 19.2 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø3.7 m) | 13.30 | 11.1% | 11.12 | 101.40 | 11.0% | ✔ |

### Mk V «Manta» [n_cells=3] — structure @ tow 18.0 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø3.7 m) | 22.17 | 18.5% | 10.37 | 101.40 | 10.2% | ✔ |

### Mk V «Manta» [n_cells=7] — structure @ tow 18.0 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø3.7 m) | 9.50 | 7.9% | 10.37 | 101.40 | 10.2% | ✔ |

### Mk V «Manta» [pressure_bar=0.07] — structure @ tow 18.0 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø3.7 m) | 9.31 | 7.8% | 10.37 | 70.98 | 14.6% | ✔ |

### Mk V «Manta» [pressure_bar=0.14] — structure @ tow 18.0 kN (12 m/s)
| Member | Hoop [kN/m] | Hoop util | M_applied [kN·m] | M_wrinkle [kN·m] | Bending util | OK |
|---|---|---|---|---|---|---|
| fat-wing box (equiv Ø3.7 m) | 18.62 | 15.5% | 10.37 | 141.97 | 7.3% | ✔ |

## Flags

- **Mk V «Manta»**: no L0 flags
- **Mk V «Manta» [chord=11.5]**: no L0 flags
- **Mk V «Manta» [chord=15.0]**: no L0 flags
- **Mk V «Manta» [taper=0.25]**: no L0 flags
- **Mk V «Manta» [taper=0.45]**: no L0 flags
- **Mk V «Manta» [n_cells=3]**: no L0 flags
- **Mk V «Manta» [n_cells=7]**: no L0 flags
- **Mk V «Manta» [pressure_bar=0.07]**: no L0 flags
- **Mk V «Manta» [pressure_bar=0.14]**: no L0 flags