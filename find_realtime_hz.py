import mujoco
import numpy as np
import time
import os

MODEL_PATH = os.path.join("mujoco_menagerie", "trs_so_arm100", "scene.xml")

# List of timesteps to try (in seconds)
timesteps = [0.001, 0.002, 0.005, 0.01, 0.02, 0.033, 0.05, 0.1]

print("Testing timesteps (Hz):")
print([1/t for t in timesteps])

results = []

for ts in timesteps:
    # Load model and set timestep
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    model.opt.timestep = ts
    data = mujoco.MjData(model)
    
    # Simulate for 5 seconds of sim time
    sim_steps = int(5.0 / ts)
    start = time.time()
    for _ in range(sim_steps):
        mujoco.mj_step(model, data)
    elapsed = time.time() - start
    if elapsed > 0:
        realtime_factor = 5.0 / elapsed
        print(f"timestep={ts:.4f} ({1/ts:.1f} Hz): sim 5s in {elapsed:.2f}s (realtime factor: {realtime_factor:.2f}x)")
    else:
        realtime_factor = float('inf')
        print(f"timestep={ts:.4f} ({1/ts:.1f} Hz): sim 5s in <0.01s (realtime factor: >500x)")
    results.append({'timestep': ts, 'hz': 1/ts, 'elapsed': elapsed, 'realtime_factor': realtime_factor})

print("\nA realtime factor >= 1.0 means the simulation is running in real time or faster.")
print("Pick the largest timestep (lowest Hz) where the realtime factor is still >= 1.0. This is the slowest simulation that still runs in real time, and will be the most efficient for your computer. Set model.opt.timestep to this value in your code.")

# Find the best value
best = None
for r in results:
    if r['realtime_factor'] >= 1.0:
        if best is None or r['timestep'] > best['timestep']:
            best = r
if best:
    print(f"\nRecommended: timestep={best['timestep']:.4f} ({best['hz']:.1f} Hz) for real-time sim (realtime factor: {best['realtime_factor']:.2f}x)")
    print(f"Set model.opt.timestep = {best['timestep']:.4f} in your code.")
else:
    print("\nNo timestep found that matches real time or faster. Try a larger timestep or a simpler model.") 