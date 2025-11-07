# Analysis metric pipeline

## 개요

로봇 제어 에피소드 동안 발생하는 모든 물리량(torque, velocity, acceleration, jerk, energy, life ratio)을 자동으로 수집, 저장, 분석.

### 데이터 흐름

```
에피소드 실행 (eval.py)
    ↓
매 제어 스텝마다 메트릭 수집
    - Torque (평균 & 현재값)
    - Velocity, Acceleration
    - Jerk (DeltaBuffer delta 방식)
    - Electric Energy (부호 유지)
    ↓
에피소드 종료
    ↓
기본 메트릭 저장 (basic_metric.py)
    - 8가지 시계열 데이터를 npy 파일로 저장
    - Gradient 방식 jerk 계산
    - Energy 분리 (draw/regen/net/total_abs)
    ↓
추가 메트릭 계산 (extra_metric.py)
    - 각 메트릭별 통계 계산 (mean/max/min/sum/std)
    - Joint별 상세 통계
    - Energy 분류 (draw/regen/net)
    ↓
CSV 파일 저장 (timeseries 폴더)
    - 모든 통계 및 energy 분석을 csv로 저장
    
비교 분석 (compare_metrics.py - 선택사항)
    - 두 조건(before/after)의 torque_current 비교
    - Rainflow-Miner로 손상(damage) 계산
    - Life Ratio (Da / Db) 계산 및 저장
```

### 주요 개념

| 개념 | 설명 | 용도 |
|------|------|------|
| **DeltaBuffer** | 이전값과 현재값을 유지하는 2-길이 버퍼 | delta/average 빠른 계산 |
| **Torque Average** | (τ_prev + τ_curr) / 2 | 에너지 계산 (부드러운 변화) |
| **Torque Current** | τ_curr (현재값만) | Life Ratio 계산 (최대 응력 반영) |
| **Jerk (Delta)** | (값_current - 값_prev) / dt | 순간 변화율 추정 |
| **Jerk (Gradient)** | np.gradient(전체_history, dt) | 전체 추세 기반 변화율 |
| **Damage** | Σ(amp^m × count) via Rainflow-Miner | 피로 누적 손상도 |
| **Life Ratio** | D_after / D_before | 두 조건 간 상대 피로 비율 |
| **Energy Draw** | Σ max(E, 0) | 모터가 소비한 에너지 |
| **Energy Regen** | Σ abs(min(E, 0)) | 회생된 에너지 (절댓값) |

### 생성되는 파일

**기본 메트릭 (npy 형식):**
- `torque_*.npy` - 평균 torque (에너지 계산용)
- `torque_current_*.npy` - 현재 torque (life ratio 계산용)
- `velocity_*.npy`, `acceleration_*.npy`
- `kinematic_jerk_delta_*.npy`, `kinematic_jerk_gradient_*.npy`
- `actuator_jerk_delta_*.npy`, `actuator_jerk_gradient_*.npy`
- `electric_energy_*.npy` - 부호 유지한 전기 에너지

**분석 결과 (csv 형식, timeseries 폴더):**
- `timeseries/metrics_*.csv` - 모든 통계, energy 분석 (life ratio는 비교 분석에서 별도 계산)

### 빠른 시작
(README.md 파일 명령어 참고)

---

---

## 목차

1. [eval.py - 메트릭 수집 및 저장](#evalpy---메트릭-수집-및-저장)
2. [basic_metric.py - 기본 메트릭 저장](#basic_metricpy---기본-메트릭-저장)
3. [extra_metric.py - 추가 메트릭 계산](#extra_metricpy---추가-메트릭-계산)
4. [compare_metrics.py - 비교 분석 (선택)](#compare_metricspy---비교-분석-선택)

---

## eval.py - 메트릭 수집 및 저장

### 1️⃣ 제어 시간 추론 (dt 계산)

**위치:** `eval_libero()` → 에피소드 초기화 시

**코드:**
```python
if args.save_metrics:
    dt = infer_dt_from_env(env)
    logging.info(f"Control dt = {dt:.6f}s (≈ {1.0/dt:.2f} Hz)")
```

**로깅 출력:**
```
Control dt = 0.050000s (≈ 20.00 Hz)
```

**의미:**
- 환경의 제어 주기 자동 추론
- dt는 모든 메트릭 계산에 사용 (특히 jerk, energy 계산)
- 출력: 초 단위 + Hz 단위 (사용자 친화적)

---

### 2️⃣ DeltaBuffer 및 메트릭 리스트 초기화

**위치:** `eval_libero()` → 에피소드 시작 시

**코드:**
```python
if args.save_metrics:
    dt = infer_dt_from_env(env)
    
    # Initialize DeltaBuffers and metric lists
    n_joints = len(env.env.robots[0]._joint_indexes)
    recent_joint_acc = DeltaBuffer(dim=n_joints)
    recent_torques = DeltaBuffer(dim=n_joints)
    recent_velocities = DeltaBuffer(dim=n_joints)
    
    # Separate lists for torque (for life ratio calculation, we need current torque, not average)
    torque_avg_list = []  # For energy calculation (average of prev and current)
    torque_current_list = []  # For life ratio calculation (current step only)
```

**의미:**
- `DeltaBuffer`: 이전값과 현재값 유지 (delta/average 계산용)
- `torque_avg_list`: 에너지 계산용 (평균 torque)
- `torque_current_list`: Life Ratio 계산용 (현재 step torque)

---

### 3️⃣ 메트릭 수집 (매 제어 스텝마다)

**위치:** `eval_libero()` → 메인 while 루프 내 action 실행 후

**코드 (Torque 예시):**
```python
if args.save_metrics:
    # Torque - two versions:
    # 1) Average (for energy calculation): (prev + current) / 2
    # 2) Current (for life ratio calculation): current step only
    torque_avg = np.array(env.env.robots[0].recent_torques.average, dtype=float)
    torque_current = np.array(env.env.robots[0].recent_torques.current, dtype=float)
    
    torque_avg_list.append(torque_avg)
    torque_current_list.append(torque_current)
    recent_torques.push(torque_current)
    
    if args.debug_action:
        logging.debug(f"Torque - avg: {torque_avg}, current: {torque_current}, diff: {torque_current - torque_avg}")
```

**로깅 출력 (debug_action=True):**
```
Torque - avg: [10.5 12.3 11.8], current: [11 12 12.5], diff: [0.5 -0.3 0.7]
```

**Velocity 수집:**
```python
velocity = np.array(env.env.robots[0]._joint_velocities, dtype=float)
velocity_list.append(velocity)
recent_velocities.push(velocity)
```

**Acceleration 수집:**
```python
acc = np.array(env.sim.data.qacc[env.env.robots[0]._ref_joint_vel_indexes], dtype=float)
acceleration_list.append(acc)
recent_joint_acc.push(acc)
```

**Kinematic Jerk (DeltaBuffer delta 방식):**
```python
# Jerk = (acc_current - acc_previous) / dt
kinematic_jerk_delta = recent_joint_acc.delta / dt
kinematic_jerk_delta_list.append(kinematic_jerk_delta)
```

**Actuator Jerk (DeltaBuffer delta 방식):**
```python
# Jerk = (torque_current - torque_previous) / dt
actuator_jerk_delta = recent_torques.delta / dt
actuator_jerk_delta_list.append(actuator_jerk_delta)
```

**Electric Energy (부호 유지):**
```python
# Power = Torque_avg × Velocity_avg [W]
t_avg = recent_torques.average
v_avg = recent_velocities.average
P = np.sum(t_avg * v_avg)  # Power [W]
E = P * dt  # Energy [J]
electric_energy_list.append(E)
```

**의미:**
- 전력(Power) = 토크 × 속도
- 에너지(Energy) = 전력 × 시간간격
- 부호 유지: E > 0 (모터 소비), E < 0 (회생)

---

### 4️⃣ 메트릭 저장 (에피소드 종료 후)

**위치:** `eval_libero()` → 에피소드 종료 시

**코드:**
```python
if args.save_metrics:
    try:
        # Save basic metrics (npy files)
        compute_basic_metrics(
            torque_list=torque_avg_list if args.save_metrics else [],  # Use average for energy
            velocity_list=velocity_list,
            acceleration_list=acceleration_list,
            kinematic_jerk_delta_list=kinematic_jerk_delta_list,
            actuator_jerk_delta_list=actuator_jerk_delta_list,
            electric_energy_list=electric_energy_list,
            dt=dt,
            output_dir=analysis_out_path,
            task_segment=task_segment,
            episode_idx=episode_idx,
            suffix=suffix
        )
        logging.info(f"Basic metrics saved for episode {episode_idx}")
        
        # Also save current torque for life ratio calculation
        if args.save_metrics and len(torque_current_list) > 0:
            torque_current_array = np.array(torque_current_list)
            pathlib.Path(analysis_out_path).mkdir(parents=True, exist_ok=True)
            np.save(
                pathlib.Path(analysis_out_path) / f"torque_current_{task_segment}_{episode_idx}_{suffix}.npy",
                torque_current_array
            )
            logging.info(f"Saved torque_current: shape {torque_current_array.shape}")
```

**로깅 출력:**
```
Basic metrics saved for episode 0
Saved torque_current: shape (500, 9)
```

---

## basic_metric.py - 기본 메트릭 저장

### 1️⃣ dt 추론 및 로깅

**함수:** `infer_control_dt(env)`

**코드:**
```python
logging.info(f"[dt] Starting inference with {len(candidates)} candidate objects")

for i, obj in enumerate(candidates):
    if obj is None:
        logging.info(f"[dt] Candidate {i} is None, skipping")
        continue
    
    logging.info(f"[dt] Checking candidate {i}: {type(obj).__name__}")
    
    if hasattr(obj, "control_timestep"):
        dt = float(obj.control_timestep)
        dt_candidates.append(("control_timestep", dt, type(obj).__name__))
        logging.info(f"[dt] Found control_timestep={dt:.6f}s from {type(obj).__name__}")
    
    if hasattr(obj, "control_freq"):
        dt = 1.0 / float(obj.control_freq)
        dt_candidates.append(("control_freq", dt, type(obj).__name__))
        logging.info(f"[dt] Found control_freq={1.0/dt:.1f}Hz -> dt={dt:.6f}s from {type(obj).__name__}")

# 우선순위에 따라 선택
for priority in priority_order:
    for source, dt, obj_type in dt_candidates:
        if source.startswith(priority):
            logging.info(f"[dt] SUCCESS: Selected {source}={dt:.6f}s from {obj_type} (highest priority)")
            return dt

logging.error("[dt] FAILED: No dt values found, using default dt=0.05s (20 Hz)")
return 0.05
```

**로깅 출력:**
```
[dt] Starting inference with 4 candidate objects
[dt] Checking candidate 0: OffScreenRenderEnv
[dt] Found control_freq=20.0Hz -> dt=0.050000s from OffScreenRenderEnv
[dt] Found control_freq=20.0Hz -> dt=0.050000s from MuJocoEnv
[dt] Found {len(dt_candidates)} dt candidates:
[dt]   control_freq: 0.050000s from OffScreenRenderEnv
[dt] SUCCESS: Selected control_freq=0.050000s from OffScreenRenderEnv (highest priority)
```

---

### 2️⃣ 메트릭 파일 저장 및 로깅

**함수:** `compute_basic_metrics(...)`

**Torque 저장:**
```python
if len(torque_list) > 0:
    torque_array = np.array(torque_list)  # shape (T, n_joints)
    np.save(output_dir / f"torque_{task_segment}_{episode_idx}_{suffix}.npy", torque_array)
    metrics["torque"] = torque_array
    logging.info(f"Saved torque: shape {torque_array.shape}")
```

**로깅 출력:**
```
Saved torque: shape (500, 9)
```

**Velocity 저장:**
```python
if len(velocity_list) > 0:
    velocity_array = np.array(velocity_list)
    np.save(output_dir / f"velocity_{task_segment}_{episode_idx}_{suffix}.npy", velocity_array)
    metrics["velocity"] = velocity_array
    logging.info(f"Saved velocity: shape {velocity_array.shape}")
```

**로깅 출력:**
```
Saved velocity: shape (500, 9)
```

**Acceleration 저장:**
```python
if len(acceleration_list) > 0:
    acceleration_array = np.array(acceleration_list)
    np.save(output_dir / f"acceleration_{task_segment}_{episode_idx}_{suffix}.npy", acceleration_array)
    metrics["acceleration"] = acceleration_array
    logging.info(f"Saved acceleration: shape {acceleration_array.shape}")
```

**로깅 출력:**
```
Saved acceleration: shape (500, 9)
```

**Kinematic Jerk - Delta 방식:**
```python
if len(kinematic_jerk_delta_list) > 0:
    kinematic_jerk_delta_array = np.array(kinematic_jerk_delta_list)
    np.save(output_dir / f"kinematic_jerk_delta_{task_segment}_{episode_idx}_{suffix}.npy", kinematic_jerk_delta_array)
    metrics["kinematic_jerk_delta"] = kinematic_jerk_delta_array
    logging.info(f"Saved kinematic_jerk_delta: shape {kinematic_jerk_delta_array.shape}")
```

**로깅 출력:**
```
Saved kinematic_jerk_delta: shape (500, 9)
```

**Kinematic Jerk - Gradient 방식:**
```python
if len(acceleration_list) > 1:
    kinematic_jerk_gradient = compute_kinematic_jerk_gradient(acceleration_list, dt)
    if kinematic_jerk_gradient is not None:
        np.save(output_dir / f"kinematic_jerk_gradient_{task_segment}_{episode_idx}_{suffix}.npy", kinematic_jerk_gradient)
        metrics["kinematic_jerk_gradient"] = kinematic_jerk_gradient
        logging.info(f"Saved kinematic_jerk_gradient: shape {kinematic_jerk_gradient.shape}")
```

**로깅 출력:**
```
Saved kinematic_jerk_gradient: shape (500, 9)
```

**Actuator Jerk - Delta 방식:**
```python
if len(actuator_jerk_delta_list) > 0:
    actuator_jerk_delta_array = np.array(actuator_jerk_delta_list)
    np.save(output_dir / f"actuator_jerk_delta_{task_segment}_{episode_idx}_{suffix}.npy", actuator_jerk_delta_array)
    metrics["actuator_jerk_delta"] = actuator_jerk_delta_array
    logging.info(f"Saved actuator_jerk_delta: shape {actuator_jerk_delta_array.shape}")
```

**로깅 출력:**
```
Saved actuator_jerk_delta: shape (500, 9)
```

**Actuator Jerk - Gradient 방식:**
```python
if len(torque_list) > 1:
    actuator_jerk_gradient = compute_actuator_jerk_gradient(torque_list, dt)
    if actuator_jerk_gradient is not None:
        np.save(output_dir / f"actuator_jerk_gradient_{task_segment}_{episode_idx}_{suffix}.npy", actuator_jerk_gradient)
        metrics["actuator_jerk_gradient"] = actuator_jerk_gradient
        logging.info(f"Saved actuator_jerk_gradient: shape {actuator_jerk_gradient.shape}")
```

**로깅 출력:**
```
Saved actuator_jerk_gradient: shape (500, 9)
```

**Electric Energy 저장 및 에너지 분석:**
```python
if len(electric_energy_list) > 0:
    electric_energy_array = np.array(electric_energy_list)
    np.save(output_dir / f"electric_energy_{task_segment}_{episode_idx}_{suffix}.npy", electric_energy_array)
    metrics["electric_energy"] = electric_energy_array
    logging.info(f"Saved electric_energy: shape {electric_energy_array.shape}")
    
    # 부호 분석
    draw_energy = np.sum(np.maximum(electric_energy_array, 0))      # 모터 소비 에너지 [J]
    regen_energy = np.sum(np.abs(np.minimum(electric_energy_array, 0)))  # 회생 에너지 [J]
    net_energy = np.sum(electric_energy_array)                       # 순 에너지 [J]
    total_absolute_energy = draw_energy + regen_energy              # 총 에너지 소비 [J]
    
    logging.info(f"Energy Summary: draw={draw_energy:.6f}J, regen={regen_energy:.6f}J, net={net_energy:.6f}J, total_abs={total_absolute_energy:.6f}J")
```

**로깅 출력:**
```
Saved electric_energy: shape (500,)
Energy Summary: draw=150.234567J, regen=45.123456J, net=105.111111J, total_abs=195.357023J
```

**에너지 분석 항목:**
| 항목 | 의미 | 계산 |
|------|------|------|
| draw | 모터 소비 에너지 | Σ max(E, 0) |
| regen | 회생 에너지 | Σ \|min(E, 0)\| |
| net | 순 에너지 (부호 유지) | Σ E |
| total_abs | 총 에너지 소비 | draw + regen |

---

## extra_metric.py - 추가 메트릭 계산

### 1️⃣ Torque 통계 계산

**함수:** `compute_extra_metrics(...)`

**코드:**
```python
if torque_avg_file.exists():
    torque_avg_array = np.load(torque_avg_file)
    metrics["torque_average"] = compute_statistics(torque_avg_array, "torque_average")
    logging.info(f"Computed torque_average statistics")

if torque_current_file.exists():
    torque_current_array = np.load(torque_current_file)
    metrics["torque_current"] = compute_statistics(torque_current_array, "torque_current")
    logging.info(f"Computed torque_current statistics")
```

**로깅 출력:**
```
Computed torque_average statistics
Computed torque_current statistics
```

### 2️⃣ Torque Current 통계 (비교 분석용)

**코드:**
```python
# Load current torque for statistics (life ratio calculation moved to compare_metrics.py)
if torque_current_file.exists():
    torque_current_array = np.load(torque_current_file)
    metrics["torque_current"] = compute_statistics(torque_current_array, "torque_current")
    logging.info(f"Computed torque_current statistics")
```

**로깅 출력:**
```
Computed torque_current statistics
```

**참고:** Life Ratio(손상도 비교)는 `compare_metrics.py`에서 두 조건을 비교할 때 계산됩니다.

### 3️⃣ 통계 계산

**함수:** `compute_statistics(arr, metric_name)`

**특징:**
- 1D 배열: 전체 통계 계산 (mean, max, min, sum, std)
- 2D 배열: 전체 + 관절별(per-joint) 통계 계산

**코드:**
```python
if arr.ndim == 1:
    # 1D array (에너지 등)
    stats["mean"] = float(np.mean(arr))
    stats["max"] = float(np.max(arr))
    stats["min"] = float(np.min(arr))
    stats["sum"] = float(np.sum(arr))
    stats["std"] = float(np.std(arr))
else:
    # 2D array (torque, velocity 등) - 관절별 계산
    stats["mean"] = float(np.mean(arr))
    stats["max"] = float(np.max(arr))
    stats["min"] = float(np.min(arr))
    stats["sum"] = float(np.sum(arr))
    stats["std"] = float(np.std(arr))
    
    # Per-joint statistics
    for joint_idx in range(arr.shape[1]):
        joint_data = arr[:, joint_idx]
        stats[f"joint_{joint_idx}_mean"] = float(np.mean(joint_data))
        stats[f"joint_{joint_idx}_max"] = float(np.max(joint_data))
        stats[f"joint_{joint_idx}_min"] = float(np.min(joint_data))
```

**통계 항목 (예: torque):**
```
torque:
  mean: 10.5234
  max: 15.7890
  min: 5.1234
  sum: 5261.7000
  std: 2.3456
  joint_0_mean: 10.1234
  joint_0_max: 14.5678
  joint_0_min: 6.2345
  joint_1_mean: 10.9235
  ...
```

### 4️⃣ Energy 분리 (draw/regen/net)

**코드:**
```python
if electric_energy_file.exists():
    electric_energy_array = np.load(electric_energy_file)
    
    # Separate positive (draw) and negative (regen) energy
    draw_energy = np.maximum(electric_energy_array, 0.0)      # E > 0: 모터 소비
    regen_energy = np.abs(np.minimum(electric_energy_array, 0.0))  # E < 0: 회생 (절댓값)
    net_energy = electric_energy_array  # 부호 유지
    
    metrics["energy_draw"] = compute_statistics(draw_energy, "energy_draw")
    metrics["energy_regen"] = compute_statistics(regen_energy, "energy_regen")
    metrics["energy_net"] = compute_statistics(net_energy, "energy_net")
    
    # Summary
    total_draw = float(np.sum(draw_energy))
    total_regen = float(np.sum(regen_energy))
    total_net = float(np.sum(net_energy))
    total_absolute = total_draw + total_regen  # 총 에너지 변화
    
    logging.info(f"Energy Summary: draw={total_draw:.6f}J, regen={total_regen:.6f}J, net={total_net:.6f}J, total_abs={total_absolute:.6f}J")
```

**로깅 출력:**
```
Energy Summary: draw=150.234567J, regen=45.123456J, net=105.111111J, total_abs=195.357023J
```

### 5️⃣ CSV 저장 (timeseries 폴더)

**함수:** `save_metrics_to_csv(...)`

**코드:**
```python
def save_metrics_to_csv(all_metrics, task_segment, episode_idx, suffix, output_dir, task_description=""):
    output_dir = pathlib.Path(output_dir)
    timeseries_dir = output_dir / "timeseries"  # timeseries 하위 폴더
    timeseries_dir.mkdir(parents=True, exist_ok=True)
    
    csv_file = timeseries_dir / f"metrics_{task_segment}_{episode_idx}_{suffix}.csv"
    
    try:
        with open(csv_file, "w", newline="") as f:
            writer = csv.writer(f)
            
            # Header
            writer.writerow(["metric_type", "metric_name", "value"])
            
            # Data
            for metric_type, metric_dict in all_metrics.items():
                for metric_name, value in metric_dict.items():
                    writer.writerow([metric_type, metric_name, value])
        
        logging.info(f"Saved metrics to {csv_file}")
        return str(csv_file)
    
    except Exception as e:
        logging.error(f"Failed to save metrics to CSV: {e}")
        return ""
```

**로깅 출력:**
```
Saved metrics to /path/to/analysis/analysis_push_button_20251106_120000/timeseries/metrics_push_button_0_success.csv
```

**CSV 파일 구조:**
```csv
metric_type,metric_name,value
torque_average,mean,10.5234
torque_average,max,15.7890
torque_average,min,5.1234
torque_average,sum,5261.7000
torque_average,std,2.3456
torque_average,joint_0_mean,10.1234
...
torque_current,mean,11.2345
torque_current,max,16.8901
...
energy_draw,mean,15.2345
energy_draw,max,28.5634
energy_regen,mean,4.5123
energy_regen,max,12.3456
energy_net,mean,10.7222
...
```

**참고:** Life Ratio는 별도로 `compare_metrics.py`에서 계산

---

## 📊 전체 로깅 흐름도

```
1. 에피소드 실행 (eval.py - --save_metrics true)
   ├── dt 추론 → logging.info(f"Control dt = {dt:.6f}s")
   ├── 매 스텝 메트릭 수집
   │   ├── torque_avg, torque_current → logging.debug (if debug_action)
   │   ├── velocity, acceleration → append
   │   ├── jerk (delta) → append
   │   └── electric_energy → append
   └── 에피소드 종료
       ├── basic_metric.py 호출
       │   ├── compute_kinematic_jerk_gradient() → logging.info(...)
       │   ├── compute_actuator_jerk_gradient() → logging.info(...)
       │   ├── 각 메트릭 저장 (npy) → logging.info("Saved {metric}: shape")
       │   └── Energy 분석 → logging.info("Energy Summary: draw=X, regen=Y, net=Z, total_abs=W")
       └── extra_metric.py 호출
           ├── compute_statistics() → logging.info("Computed {metric} statistics")
           ├── energy 분리 (draw/regen/net) → logging.info("Energy Summary")
           └── save_metrics_to_csv() → logging.info("Saved metrics to timeseries/...")

2. 비교 분석 (compare_metrics.py - 선택사항)
   ├── before_dir & after_dir 찾기
   ├── 각 파일 쌍에 대해:
   │   ├── calculate_damage() → logging.info("Joint X: Damage = Y")
   │   └── compare_conditions() → logging.info("Life ratio: Z×")
   └── save_comparison_to_csv() → logging.info("✅ Comparison results saved to ...")
```

---

## 🎯 사용 방법

**메트릭 저장 활성화:**
```bash
python eval.py \
  --save_metrics true \
  --action_noise_scale 0.6 \
  --action_noise_dim "action.eef_pos_delta[2]"
```

**디버그 로깅 활성화:**
```bash
python eval.py \
  --save_metrics true \
  --debug_action true \
  --action_noise_scale 0.6 \
  --action_noise_dim "action.eef_pos_delta[2]"
```

---

## 📁 생성되는 파일 구조

```
./analysis/analysis_libero_10_20251106_120000/
├── timeseries/                                       # CSV 결과 (추가 메트릭)
│   └── metrics_{task}_{episode}_{suffix}.csv         # 통계 & energy 분석
├── torque_{task}_{episode}_{suffix}.npy              # 평균 torque (에너지 계산용)
├── torque_current_{task}_{episode}_{suffix}.npy      # 현재 torque (비교 분석용)
├── velocity_{task}_{episode}_{suffix}.npy
├── acceleration_{task}_{episode}_{suffix}.npy
├── kinematic_jerk_delta_{task}_{episode}_{suffix}.npy
├── kinematic_jerk_gradient_{task}_{episode}_{suffix}.npy
├── actuator_jerk_delta_{task}_{episode}_{suffix}.npy
├── actuator_jerk_gradient_{task}_{episode}_{suffix}.npy
└── electric_energy_{task}_{episode}_{suffix}.npy     # 부호 유지한 에너지

비교 분석 결과 (compare_metrics.py 실행 후):
./results/
└── comparison_libero_10.csv                          # 손상도 & Life Ratio 비교
```

---

---

## compare_metrics.py - 비교 분석 (선택)

### 📊 목적

두 조건(noise 없음 vs 있음) 간의 **상대적 피로 손상도(Life Ratio)** 및 **물리량 비교**를 수행합니다.
- **NPY 비교**: Rainflow-Miner로 각 조건의 손상도(Damage) 계산 → Life Ratio
- **CSV 비교**: 전체 물리량 (에너지, 토크, 가속도, 저크 등) 통계 비교

### 🚀 사용 방법

```bash
# NPY + CSV 모두 비교 (권장)
python compare_metrics.py \
  --before_dir ./analysis/analysis_libero_10_noise_00000/ \
  --after_dir ./analysis/analysis_libero_10_noise_05000_dim_action.eef_pos_delta[2]/ \
  --m 3.0

# CSV만 비교
python compare_metrics.py \
  --before_csv ./analysis/metrics_before.csv \
  --after_csv ./analysis/metrics_after.csv
```

**파라미터:**
- `--before_dir`, `--after_dir`: NPY 파일 비교용 (torque_current_*.npy 포함)
- `--m`: Basquin 지수 (기본: 3.0, 범위: 3~5)
- `--before_csv`, `--after_csv`: CSV 파일 비교용

### 📈 주요 함수

#### 1️⃣ `calculate_damage(torque_array, m=3.0, use_goodman=True, ...)`

**Goodman 보정이 적용된** 피로 손상도 계산

```python
def calculate_damage(torque_array, m=3.0, use_goodman=True, sigma_ult=500.0):
    """
    Rainflow-Miner 기반 손상 계산 (Goodman 보정)
    
    Goodman 선도: σ_f = σ_a × σ_ult / (σ_ult - σ_m)
    - σ_a: 응력 진폭 (range/2)
    - σ_m: 평균 응력 (전체 신호 평균)
    - σ_ult: 인장 강도 (500 MPa)
    
    Miner의 규칙: D = Σ(C × amplitude^m × count)
    - C=1.0: 모든 사이클을 완전 사이클로 처리 (보수적)
    """
```

**로깅 출력:**
```
Joint 0: 사이클=57, 총카운트=54.5, 최대진폭=6.93, Goodman=True, 손상=3.42e+02
손상 계산 완료: total=1.76e+04
```

#### 2️⃣ `compare_npy_metrics(before_dir, after_dir, m=3.0)`

NPY 파일 기반 피로 손상 비교

**출력 예:**
```
📊 NPY 기반 전체 요약 통계:
  평균 수명비: 471.7465x
  최소 수명비: 74.5557x (가장 양호)
  최대 수명비: 1643.3921x (가장 악화)
  비교된 파일: 165개

⚙️ Joint별 분석:
Joint_0: 손상비 5.1074x (수명 크게 단축)
Joint_1: 손상비 587.1621x (⚠️ 가장 취약)
...

📁 매칭된 파일 165개 발견
  torque_current_task_0_success.npy | 수명비: 621.4040 (단축 ⬆️)
  ...
```

#### 3️⃣ `compare_csv_metrics(before_csv, after_csv)`

CSV 파일 기반 물리량 비교

**비교 항목:**
- 🔋 Energy (draw, regen, net)
- ⚙️ Torque (average, current)
- 🏃 Kinematics (acceleration, velocity)
- ⚡ Jerk (kinematic_delta, actuator_delta 등)

**출력 예:**
```
🔋 Energy
  energy_draw.mean: -11.4% ↓
  torque_current.max: +113.5% ↑

⚡ Joint별 실제 토크 분석 (torque_current):
Joint_0: mean -19.3%, max +46.7%
Joint_1: mean -9.1%, max +245.0% (⚠️ 심각)
```

### 📊 로깅 출력

```
INFO: Step 1/2: NPY 기반 비교 시작...
INFO: 📊 NPY 기반 전체 요약 통계 (모든 파일 종합):
INFO:   평균 수명비: 471.7465x
INFO:   ⚠️ 결론: 노이즈로 인해 평균 손상이 약 47074.6% 증가 → 수명 단축

INFO: Step 2/2: CSV 기반 비교 시작...
INFO: ✅ CSV 파일 발견, 비교 시작...
INFO: ✅ CSV 비교 완료

INFO: ✅ 전체 비교 결과 저장 완료: results/comparison_*.txt (74,651 bytes)
```

### 🎯 해석 예

**결과 해석:**
- **Life Ratio = 471.75x**: 손상도가 472배 증가 → 수명이 약 99.8% 단축
- **Joint_1**: 가장 취약한 조인트 (587배 손상 증가)
- **torque_current.max**: 113.5% 증가 → 최대 응력이 2배 이상
- **Goodman 보정**: 평균 응력의 영향을 고려한 보수적 평가

---

**마지막 업데이트:** 2025-11-06

