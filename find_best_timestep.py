import mujoco
import os
import time
import numpy as np

def find_best_timestep(model_path, timesteps=None, sim_seconds=5.0):
    # 1. Coarse search
    coarse_timesteps = [0.001, 0.002, 0.005, 0.01, 0.02, 0.033, 0.05, 0.1, 0.2, 0.5, 1.0]
    results = []
    for ts in coarse_timesteps:
        model = mujoco.MjModel.from_xml_path(model_path)
        model.opt.timestep = ts
        data = mujoco.MjData(model)
        sim_steps = int(sim_seconds / ts)
        start = time.time()
        for _ in range(sim_steps):
            mujoco.mj_step(model, data)
        elapsed = time.time() - start
        if elapsed > 0:
            realtime_factor = sim_seconds / elapsed
        else:
            realtime_factor = float('inf')
        results.append({'timestep': ts, 'hz': 1/ts, 'elapsed': elapsed, 'realtime_factor': realtime_factor})
    # Find best coarse
    best = None
    for r in results:
        if r['realtime_factor'] >= 1.0:
            if best is None or r['timestep'] > best['timestep']:
                best = r
    # 2. Fine search around best coarse
    if best:
        fine_start = max(best['timestep'] - 0.1, 0.001)
        fine_end = min(best['timestep'] + 0.1, 1.0)
        fine_timesteps = [round(x, 4) for x in np.arange(fine_start, fine_end, 0.0005)]
        fine_results = []
        for ts in fine_timesteps:
            model = mujoco.MjModel.from_xml_path(model_path)
            model.opt.timestep = ts
            data = mujoco.MjData(model)
            sim_steps = int(sim_seconds / ts)
            start = time.time()
            for _ in range(sim_steps):
                mujoco.mj_step(model, data)
            elapsed = time.time() - start
            if elapsed > 0:
                realtime_factor = sim_seconds / elapsed
            else:
                realtime_factor = float('inf')
            fine_results.append({'timestep': ts, 'hz': 1/ts, 'elapsed': elapsed, 'realtime_factor': realtime_factor})
        # Find best fine
        best_fine = None
        for r in fine_results:
            if r['realtime_factor'] >= 1.0:
                if best_fine is None or r['timestep'] > best_fine['timestep']:
                    best_fine = r
        if best_fine:
            return best_fine, fine_results
        else:
            return best, results
    else:
        return None, results 