
# Testing Synchronization Simulation Speed with Unreal Visualization

## Problem

- MuJoCo simulation (`mj_step()`) **does not guarantee real-time** execution.  
- Unreal Engine renders frames at **real-time pace** (e.g., 60fps).
- Without adjustment, the **visualized motion** in Unreal **won't match real-world timing**.
- If MuJoCo is faster or slower, we must **scale Unreal’s time** to match it.

---

## Key Concepts

| Concept | Description |
| :------ | :---------- |
| `timestep` (`mjModel.opt.timestep`) | Simulation step size in seconds (e.g., `0.002` = 500Hz). |
| `simulated_time` | MuJoCo internal clock (`mjData.time`). |
| `real_time_passed` | Real-world clock elapsed time. |
| `simulation_speed` | Ratio of MuJoCo simulated time to real-world time.<br>`simulation_speed = simulated_time / real_time_passed` |

> **Goal:** Keep `simulation_speed ≈ 1.0` for realistic visualization.

---

## Setup Instructions

### 1. Set a Good MuJoCo Timestep

- Choose `mjModel.opt.timestep` based on your robot’s control loop frequency.
- Recommended: `0.001` (1kHz) or `0.002` (500Hz).
  
Example:
```cpp
model.opt.timestep = 0.002;  // 500Hz
```

---

### 2. Measure Simulation Speed

Track two times:
- **MuJoCo Time:** `mjData.time`
- **Real Clock Time:** Use high-res system timer (e.g., `std::chrono::steady_clock`).

Calculate each frame:
```cpp
simulation_speed = (mjData.time - mjDataStart) / (realTimeNow - realTimeStart)
```

Interpretation:
- `simulation_speed > 1.0` → MuJoCo is faster than real-time → Speed up Unreal.
- `simulation_speed < 1.0` → MuJoCo is slower than real-time → Slow down Unreal.

---

### 3. Adjust Unreal Engine Time

Use Unreal’s **Global Time Dilation** to sync:

```cpp
UGameplayStatics::SetGlobalTimeDilation(GetWorld(), simulation_speed);
```

| MuJoCo Speed | Unreal Time Dilation |
| :----------- | :------------------- |
| 2.0x faster | 2.0 |
| 0.5x slower | 0.5 |

---

### 4. Handle Skip Frames / Multiple Steps

- Unreal runs at ~60fps.
- MuJoCo might need **multiple steps per Unreal frame**.
- Example: if MuJoCo timestep = 0.002s (500Hz), and Unreal is 60fps:
  - `0.002s * 8 steps = 0.016s` → matches ~60Hz frame time.

Typical Unreal frame update:
```cpp
for (int i = 0; i < numMuJoCoStepsPerFrame; ++i)
    mj_step(model, data);
```

---

## Full Quick Checklist

| Parameter
