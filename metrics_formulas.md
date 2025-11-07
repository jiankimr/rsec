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

**Note**: *Regen energy is logged **separately** as a positive quantity ($|E_t|$ for $E_t<0$).*

---

## Fatigue Damage & Life Ratio (from torque_current)

### Rainflow–Miner damage
For each joint $j$, rainflow yields amplitude–count pairs $\{(A_{k,j},N_{k,j})\}_k$.
With Basquin exponent $m$ and (optional) Goodman correction $A'_{k,j}$:
$$
D=\sum_{j=1}^{n}\sum_{k} N_{k,j} \cdot \bigl(A^{(\prime)}_{k,j}\bigr)^m
$$

### Life Ratio between two conditions
With damages $D_{\text{before}},D_{\text{after}}$ computed **from $\boldsymbol{\tau}^{\text{cur}}$**:
$$
\text{Life Ratio}=\frac{D_{\text{after}}}{D_{\text{before}}}
$$
Interpretation example: if $\text{Life Ratio}=471.75$, the *remaining* life fraction is $\approx 1/471.75\approx 0.212\%$, i.e., a **$\sim 99.8\%$** reduction in life compared to baseline for the same exposure.

---

## Summary (Storage Rules)

- **Energy**: use $\boldsymbol{\tau}^{\text{avg}}, \mathbf{v}^{\text{avg}}$ → $P_t, E_t$ (signed), and **log regen separately**.
- **Life Ratio**: use $\boldsymbol{\tau}^{\text{cur}}$ → rainflow–Miner $D$ → $D_{\text{after}}/D_{\text{before}}$.
- **Jerk**: kinematic $=\dot{\mathbf{a}}$, actuator $=\dot{\boldsymbol{\tau}}$. Store Delta + Gradient variants.

