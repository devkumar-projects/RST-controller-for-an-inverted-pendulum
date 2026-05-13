# RST-controller-for-an-inverted-pendulum
# 🔵 RST Controller — Inverted Pendulum Sinusoidal Tracking

> **Robust Control Project** · ENSAM Paris · Mechatronics UE · Semester 9

[![MATLAB](https://img.shields.io/badge/MATLAB-R2024-orange?logo=mathworks&logoColor=white)](https://www.mathworks.com/)
[![Simulink](https://img.shields.io/badge/Simulink-R2024-blue?logo=mathworks&logoColor=white)](https://www.mathworks.com/products/simulink.html)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Status](https://img.shields.io/badge/Status-Completed-brightgreen)]()

-----

## 📋 Overview

This project implements a **RST pole-placement controller** designed to make the tip of an inverted pendulum track a **sinusoidal reference** while rejecting **step disturbances**, using a linearised state-space model derived from Lagrange mechanics.

The pendulum is an inherently unstable system with a real unstable open-loop pole. The RST structure is extended to embed the **internal model principle** — the polynomial `S(s)` incorporates a factor `(s² + ω₀²)` to guarantee zero steady-state tracking error at the sinusoidal frequency.

-----

## 🎯 Specifications

|Criterion               |Requirement     |Result              |
|------------------------|----------------|--------------------|
|Control signal          |`|u| ≤ 5 V`     |✅ `|u|_max = 2.03 V`|
|Phase margin            |`≥ 40°`         |✅ `Pm = 42.8°`      |
|Disturbance rejection   |Fast, stable    |✅ `Tr ≈ 1.55 s`     |
|Robustness (L variation)|Stable for all L|✅ All 4 lengths     |

-----

## 🧠 Theoretical Background

### System Model

The inverted pendulum dynamics are derived from the Lagrange equations and linearised around the vertical equilibrium (`θ = 0`):

```
f - (M+m)ẍ = mlθ̈         (cart equation)
lθ̈ = ẍ - g·θ               (rod equation)
```

After Laplace transform and simplification (neglecting cart-rod coupling since `ω₀b ≪ ω₀c`), the simplified transfer function is:

```
        -α²
H₂(s) = ─────────      with  α = √(g/l)
        s² - α²
```

### RST Structure

```
         T(s)                 R(s)
r ──►[────────]──► + ──►[ P(s) ]──► y
                   ↑               │
                   └──────[S(s)]───┘
```

The closed-loop polynomial is:

```
A_BF(s) = A(s)·S(s) + B(s)·R(s)
```

resolved via the **Bézout identity** with iso-constrained degree condition:

```
k = n + 2   →   deg(R) = 4,  deg(S) = 4,  deg(A_BF) = 6
```

### Pole Placement

Closed-loop poles are placed according to the empirical separation rule:

```
0 < β ≪ ω₀b ≪ γ ≪ ω₀c
```

|Parameter      |Initial |Optimised    |
|---------------|--------|-------------|
|`β` (slow pole)|1 rad/s |**0.5 rad/s**|
|`γ` (fast pole)|10 rad/s|**30 rad/s** |

-----

## 📁 Repository Structure

```
.
├── README.md
├── TP2_RST_Pendule_Dev_KUMAR.m      # Main MATLAB script (RST design + margins + robustness)
├── simu_rst_pendule.mdl             # Simulink closed-loop simulation model
├── placement_poles.py               # Pole-placement visualisation (Python/Matplotlib)
└── report/
    └── RST_Controller_Inverted_Pendulum_EN.docx   # Full project report (English)
```

-----

## 🚀 Getting Started

### Prerequisites

- MATLAB R2022b or later with **Control System Toolbox**
- Simulink
- Python 3.10+ with `matplotlib` and `numpy` (for pole placement plot)

### Running the MATLAB script

```matlab
% Open MATLAB and run:
TP2_RST_Pendule_Dev_KUMAR.m
```

The script will sequentially:

1. Compute the closed-loop polynomial `A_BF` for given `β`, `γ`, `L`
1. Solve the Bézout equation → coefficients of `R(s)`, `S(s)`
1. Compute `T(s)` from the tracking conditions
1. Plot Bode and Nyquist diagrams, compute stability margins
1. Run Simulink simulation and plot time-domain responses
1. Run the robustness sweep over `L ∈ {0.16, 0.23, 0.28, 0.35}` m

### Pole placement visualisation (Python)

```bash
pip install matplotlib numpy
python placement_poles.py
```

-----

## 📊 Key Results

### Stability Analysis

|                           |Initial RST (`β=1, γ=10`)|Optimised RST (`β=0.5, γ=30`)|
|---------------------------|-------------------------|-----------------------------|
|Phase margin               |30°                      |**42.8°**                    |
|Modulus margin `Mm`        |0.49                     |improved                     |
|Max control `|u|`          |< 5 V                    |**2.03 V**                   |
|Max rod angle (disturbance)|17.56°                   |**15.05°**                   |

### Robustness (optimised RST, varying L)

|L [m]|α [rad/s]|Phase margin [°]|Disturbance Tr [s]|
|-----|---------|----------------|------------------|
|0.16 |7.83     |> 48°           |≈ 1.56            |
|0.23 |6.53     |> 48°           |≈ 1.56            |
|0.28 |5.92     |> 48°           |≈ 1.56            |
|0.35 |5.29     |> 48°           |≈ 1.56            |


> The rejection response time is quasi-invariant with L — a direct consequence of pole placement: closed-loop dominant poles are fixed by the controller, not by the process.

-----

## ⚙️ Physical Parameters

|Symbol |Value        |Description                   |
|-------|-------------|------------------------------|
|`g`    |9.81 m/s²    |Gravitational acceleration    |
|`L`    |0.16 – 0.35 m|Rod length (variable)         |
|`ω₀`   |1 rad/s      |Sinusoidal reference frequency|
|`ω₀c`  |51.66 rad/s  |Cart natural frequency        |
|`ξ`    |0.495        |Cart damping ratio            |
|`u_max`|±5 V         |Control saturation limit      |

-----

## 📖 References

- Åström, K.J. & Wittenmark, B. — *Computer-Controlled Systems: Theory and Design*
- Landau, I.D. & Zito, G. — *Digital Control Systems*
- Course notes — UE Mechatronics, ENSAM Paris, 2026

-----

## ✍️ Author

**KUMAR Dev** — ENSAM Paris, Semester 9  
Supervisor: **M. GUILLARD Hervé**

-----

*Project completed as part of the Robust Control module — Mechatronics UE, 2026.*
