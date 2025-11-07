# Analysis Metric Formulas & Logging Spec

> Last updated: 2025-11-07 (Asia/Seoul)

This document defines **actuator jerk**, **kinematic jerk**, and **torque logging** (average vs current), plus **energy** (with signed values and separate regen logging) and **Life Ratio** for fatigue analysis.

---

## Notation

- Discrete step: $t=0,1,2,\dots$, control period $\Delta t>0$.
- Vector per joint ($n$ joints): bold symbols $\in\mathbb{R}^n$.
- **DeltaBuffer** (keeps previous & current):

$$
x_t^{\text{avg}}=\frac{x_{t-1}+x_t}{2},\quad
x_t^{\text{cur}}=x_t,\quad
\Delta x_t=x_t-x_{t-1}.
$$

- Variables: $\boldsymbol{\tau}_t$ (torque), $\mathbf{v}_t$ (velocity), $\mathbf{a}_t$ (acceleration).

---

## Torque Logging (Energy vs Life)

### Energy-oriented (average torque)

$$
\boldsymbol{\tau}_t^{\text{avg}}=\frac{\boldsymbol{\tau}_{t-1}+\boldsymbol{\tau}_t}{2}
$$

Saved as: `torque_{...}.npy` (used for power/energy).

### Fatigue/Life-oriented (current torque)

$$
\boldsymbol{\tau}_t^{\text{cur}}=\boldsymbol{\tau}_t
$$

Saved as: `torque_current_{...}.npy` (used for rainflow–Miner & Life Ratio).

---

## Jerk Definitions

### Kinematic jerk (time derivative of acceleration)

- **Delta (difference) form**

$$
\mathbf{j}^{\text{kin}}_t=\frac{\mathbf{a}_t-\mathbf{a}_{t-1}}{\Delta t}
$$

- **Gradient (history) form** (via `np.gradient`)

$$
\mathbf{j}^{\text{kin,grad}}(t)\approx \frac{d\mathbf{a}}{dt}
$$

Saved as: `kinematic_jerk_delta_{...}.npy`, `kinematic_jerk_gradient_{...}.npy`.

### Actuator jerk (time derivative of torque)

- **Delta (difference) form**

$$
\mathbf{j}^{\text{act}}_t=\frac{\boldsymbol{\tau}_t-\boldsymbol{\tau}_{t-1}}{\Delta t}
$$

- **Gradient (history) form** (via `np.gradient`)

$$
\mathbf{j}^{\text{act,grad}}(t)\approx \frac{d\boldsymbol{\tau}}{dt}
$$

Saved as: `actuator_jerk_delta_{...}.npy`, `actuator_jerk_gradient_{...}.npy`.

---

## Power / Energy (Sign-preserving & Regen-split)

### Per-step power (sum across joints)

$$
P_t=\sum_{j=1}^{n}\tau_{t,j}^{\text{avg}}\,v_{t,j}^{\text{avg}}\quad [\mathrm{W}],\qquad
\mathbf{v}_t^{\text{avg}}=\frac{\mathbf{v}_{t-1}+\mathbf{v}_t}{2}.
$$

### Per-step energy (keep sign)

$$
E_t=P_t\,\Delta t\quad [\mathrm{J}]
$$

- **Sign meaning**: $E_t>0$ draw (consumed), $E_t<0$ regen (returned).

### Aggregates (explicit regen logging)

$$
E_{\text{draw}}=\sum_t \max(E_t,0),\quad
E_{\text{regen}}=\sum_t \lvert \min(E_t,0)\rvert,\quad
E_{\text{net}}=\sum_t E_t,\quad
E_{\text{total abs}}=E_{\text{draw}}+E_{\text{regen}}
$$

Saved as: `electric_energy_{...}.npy` (signed timeseries) and CSV summaries for `energy_draw`, `energy_regen`, `energy_net`, `total_abs`.

**Note**: *Regen energy is logged **separately** as a positive quantity ( $\lvert E_t \rvert$ for $E_t<0$ ).*

---

## Fatigue Damage & Life Ratio (from torque_current)

### Rainflow–Miner Damage Algorithm

#### 1. Rainflow Cycle Extraction

For each joint $j$, the rainflow algorithm extracts stress cycles from zero-mean torque history $\boldsymbol{\tau}_{j,\text{centered}}$:

$$
\boldsymbol{\tau}_{j,\text{centered}} = \boldsymbol{\tau}_j - \text{mean}(\boldsymbol{\tau}_j)
$$

Each cycle $k$ yields:
- **Range**: $R_{k,j}$ (full cycle range)
- **Amplitude**: $A_{k,j} = R_{k,j}/2$
- **Cycle mean**: $\sigma_{m,k,j}$ (mean stress during cycle)
- **Count**: $N_{k,j}$ (0.5 for half-cycle, 1.0 for full cycle)

#### 2. Goodman Mean-Stress Correction (optional)

To account for mean stress effects on fatigue life:

$$
A'_{k,j} = A_{k,j} \cdot \frac{\sigma_{\text{ult}}}{\sigma_{\text{ult}} - \sigma_{m,j}}
$$

where:
- $A'_{k,j}$ = effective amplitude (Goodman-corrected)
- $\sigma_{\text{ult}}$ = ultimate tensile strength (default: 500 MPa)
- $\sigma_{m,j}$ = mean stress of joint $j$ (absolute value of $\text{mean}(\boldsymbol{\tau}_j)$)

If Goodman correction is disabled, $A'_{k,j} = A_{k,j}$.

#### 3. Miner's Cumulative Damage Rule

For each joint, damage accumulates as:

$$
D_j = \sum_{k} C \cdot N_{k,j} \cdot (A'_{k,j})^m
$$

where:
- $C = 1.0$ (conservative constant for closed cycles)
- $m$ = Basquin exponent (default: 3.0)
- $N_{k,j}$ = cycle count
- $A'_{k,j}$ = Goodman-corrected amplitude

#### 4. Total Damage

Summing over all joints:

$$
D_{\text{total}} = \sum_{j=1}^{n} D_j
$$

### Life Ratio between two conditions

With damages $D_{\text{before}}, D_{\text{after}}$ computed **from $\boldsymbol{\tau}^{\text{cur}}$**:

$$
\text{Life Ratio}=\frac{D_{\text{after}}}{D_{\text{before}}}
$$

**Interpretation**: 
- If $\text{Life Ratio}=471.75$, the *remaining* life fraction is $\approx 1/471.75 \approx 0.212\%$.
- This represents a **$\sim 99.8\%$** reduction in fatigue life compared to baseline for the same exposure duration.
- Higher Life Ratio = more severe damage = shorter remaining life.

### Parameters Used in Implementation

See `rsec/LIBERO/analyze/compare_metrics.py`:

```python
def calculate_damage(torque_array, m=3.0, use_goodman=True, 
                     sigma_ult=500.0, sigma_y=400.0)
```

- `m=3.0`: Basquin exponent (typical range: 3–5)
- `use_goodman=True`: Apply Goodman mean-stress correction
- `sigma_ult=500.0`: Ultimate tensile strength [MPa]
- `sigma_y=400.0`: Yield strength [MPa] (for reference)

---

## Summary (Storage Rules)

- **Energy**: use $\boldsymbol{\tau}^{\text{avg}}, \mathbf{v}^{\text{avg}}$ → $P_t, E_t$ (signed), and **log regen separately**.
- **Life Ratio**: use $\boldsymbol{\tau}^{\text{cur}}$ → rainflow–Miner $D$ → $D_{\text{after}}/D_{\text{before}}$.
- **Jerk**: kinematic $=\dot{\mathbf{a}}$, actuator $=\dot{\boldsymbol{\tau}}$. Store Delta + Gradient variants.

