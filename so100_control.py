import mujoco
import numpy as np
import glfw
import os
import threading
import queue
import time
import find_best_timestep

def init_glfw():
    if not glfw.init():
        raise Exception("Failed to initialize GLFW")
    
    window = glfw.create_window(1200, 900, "SO100 Arm Control", None, None)
    if not window:
        glfw.terminate()
        raise Exception("Failed to create GLFW window")
    
    glfw.make_context_current(window)
    return window

def main():
    # Initialize GLFW
    window = init_glfw()
    
    # Path to SO100 arm scene
    model_path = os.path.join("mujoco_menagerie", "trs_so_arm100", "scene.xml")
    assets_path = os.path.join("mujoco_menagerie", "trs_so_arm100", "assets")
    
    if not os.path.exists(model_path):
        print(f"Error: Could not find SO100 model at {model_path}")
        return
    
    # Set meshdir for relative asset loading
    old_cwd = os.getcwd()
    os.chdir(os.path.dirname(model_path))
    model = mujoco.MjModel.from_xml_path(os.path.basename(model_path))
    # Find dynamic initial timestep
    try:
        best, _ = find_best_timestep.find_best_timestep(os.path.basename(model_path))
        if best:
            model.opt.timestep = best['timestep']
            print(f"[INFO] Initial timestep: {best['timestep']}")
    except Exception as e:
        print(f"[INFO] find_best_timestep could not be executed: {e}")
    os.chdir(old_cwd)
    data = mujoco.MjData(model)
    
    # Get actuator names and ranges
    actuator_names = [model.actuator(i).name for i in range(model.nu)]
    actuator_ranges = [(model.actuator(i).ctrlrange[0], model.actuator(i).ctrlrange[1]) for i in range(model.nu)]
    
    print("\nAvailable actuators:")
    for i, (name, (min_val, max_val)) in enumerate(zip(actuator_names, actuator_ranges)):
        print(f"{i}: {name} (range: {min_val:.2f} to {max_val:.2f})")
    
    # Create visualization context
    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    scn = mujoco.MjvScene(model, maxgeom=10000)
    con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    
    # Set camera
    cam.azimuth = 90
    cam.elevation = -30
    cam.distance = 1.8
    cam.lookat = np.array([0, 0, 0.1])
    
    # Store current torques for each actuator
    current_torques = np.zeros(model.nu)
    # Store target positions for each actuator (None if not set)
    target_positions = [None] * model.nu
    kp_pos = 10.0  # Proportional gain for position control
    cmd_queue = queue.Queue()
    stop_flag = threading.Event()
    realtime_sync = False
    real_time = time.time()
    timestep_update_interval = 50  # Update more frequently
    step_counter = 0
    timestep_min = 0.0001  # Reduced minimum timestep
    timestep_max = 0.1    # Reduced maximum timestep
    integral_error = 0.0  # For integral control
    auto_timestep = True  # Flag to control auto timestep adjustment
    # Adaptive step: based on delta_time magnitude
    # timestep_step = 0.0005

    # Store initial positions
    initial_positions = np.copy(data.qpos)

    # Open log file
    log_file = open("so100.log", "w")
    log_file.write("# ts, sim_time, real_time, " + ", ".join([f"torque_{name}" for name in actuator_names]) + ", " + ", ".join([f"qpos_{i}" for i in range(model.nq)]) + "\n")
    sim_time = 0.0
    start_wall_time = time.time()

    def input_thread():
        while not stop_flag.is_set():
            print("\nEnter 'torque <actuator_index> <value>' to set torque, 'pos <actuator_index> <value>' to set position, 'realtime on/off' to toggle real-time sync, 'timestep <value>' to set timestep, 'timestep auto' to enable auto-timestep, 'reset' to reset robot, 'stop' to stop all, or 'q' to quit:")
            user_input = input("> ")
            if user_input.lower() == 'q':
                stop_flag.set()
                break
            cmd_queue.put(user_input)

    t = threading.Thread(target=input_thread, daemon=True)
    t.start()

    while not glfw.window_should_close(window) and not stop_flag.is_set():
        # Process CLI commands
        while not cmd_queue.empty():
            user_input = cmd_queue.get()
            try:
                if user_input.startswith('realtime'):
                    _, state = user_input.split()
                    if state == 'on':
                        realtime_sync = True
                        print('[REALTIME] Real-time sync enabled.')
                    elif state == 'off':
                        realtime_sync = False
                        print('[REALTIME] Real-time sync disabled.')
                    else:
                        print("Usage: realtime on|off")
                elif user_input.strip() == 'reset':
                    # Reset positions to initial values
                    data.qpos[:] = initial_positions
                    # Reset velocities to zero
                    data.qvel[:] = 0
                    # Reset torques and target positions
                    current_torques[:] = 0
                    target_positions = [None] * model.nu
                    # Reset integral error for timestep control
                    integral_error = 0.0
                    print('[RESET] Robot reset to initial position, all torques cleared.')
                elif user_input.startswith('timestep'):
                    _, value = user_input.split()
                    if value.lower() == 'auto':
                        auto_timestep = True
                        print('[TIMESTEP] Auto-timestep adjustment enabled.')
                    else:
                        try:
                            new_timestep = float(value)
                            if timestep_min <= new_timestep <= timestep_max:
                                model.opt.timestep = new_timestep
                                auto_timestep = False
                                print(f'[TIMESTEP] Set to {new_timestep:.6f}')
                            else:
                                print(f'[TIMESTEP] Value must be between {timestep_min} and {timestep_max}')
                        except ValueError:
                            print('[TIMESTEP] Invalid value. Use a number or "auto"')
                elif user_input.startswith('torque'):
                    _, actuator_idx, value = user_input.split()
                    actuator_idx = int(actuator_idx)
                    value = float(value)
                    if 0 <= actuator_idx < model.nu:
                        min_val, max_val = actuator_ranges[actuator_idx]
                        value = np.clip(value, min_val, max_val)
                        current_torques[actuator_idx] = value
                        target_positions[actuator_idx] = None  # Cancel any position control
                        print(f"[TORQUE] Set actuator {actuator_names[actuator_idx]} to {value:.2f}")
                    else:
                        print(f"Invalid actuator index. Must be between 0 and {model.nu-1}")
                elif user_input.startswith('pos'):
                    _, actuator_idx, value = user_input.split()
                    actuator_idx = int(actuator_idx)
                    value = float(value)
                    if 0 <= actuator_idx < model.nu:
                        min_val, max_val = actuator_ranges[actuator_idx]
                        value = np.clip(value, min_val, max_val)
                        target_positions[actuator_idx] = value
                        print(f"[POS] Target position for joint {actuator_names[actuator_idx]} set to {value:.2f}")
                    else:
                        print(f"Invalid actuator index. Must be between 0 and {model.nu-1}")
                elif user_input.startswith('snap'):
                    _, actuator_idx, value = user_input.split()
                    actuator_idx = int(actuator_idx)
                    value = float(value)
                    if 0 <= actuator_idx < model.nu:
                        data.qpos[model.jnt_qposadr[actuator_idx]] = value
                        print(f"[SNAP] Instantly set joint {actuator_names[actuator_idx]} to {value:.2f}")
                    else:
                        print(f"Invalid actuator index. Must be between 0 and {model.nu-1}")
                elif user_input.strip() == 'stop':
                    current_torques[:] = 0
                    target_positions = [None] * model.nu
                    print("[STOP] All torques set to zero and position control cancelled.")
                else:
                    actuator_idx, value = map(float, user_input.split())
                    actuator_idx = int(actuator_idx)
                    if 0 <= actuator_idx < model.nu:
                        min_val, max_val = actuator_ranges[actuator_idx]
                        value = np.clip(value, min_val, max_val)
                        current_torques[actuator_idx] = value
                        target_positions[actuator_idx] = None
                        print(f"Set actuator {actuator_names[actuator_idx]} to {value:.2f}")
                    else:
                        print(f"Invalid actuator index. Must be between 0 and {model.nu-1}")
            except ValueError:
                print("Invalid input format. Use 'torque <actuator_index> <value>', 'pos <actuator_index> <value>', 'stop', or 'q'")

        # Apply current torques and position control
        for i in range(model.nu):
            if target_positions[i] is not None:
                qpos_addr = model.jnt_qposadr[i]
                current_pos = data.qpos[qpos_addr]
                error = target_positions[i] - current_pos
                current_torques[i] = kp_pos * error
        data.ctrl[:] = current_torques
        mujoco.mj_step(model, data)
        sim_time += model.opt.timestep

        # Real-time synchronization (optional)
        if realtime_sync:
            step_end = time.time()
            elapsed = step_end - real_time
            sleep_time = model.opt.timestep - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            real_time = time.time()

        # Log torques and joint positions with real time
        real_time = time.time() - start_wall_time
        delta_time = sim_time - real_time
        ts = time.time()
        log_line = f"ts:{model.opt.timestep:.6f}, s:{sim_time:.3f}, r:{real_time:.3f}, d:{delta_time:.3f} " + ", ".join(f"{t:.6f}" for t in current_torques) + ", " + ", ".join(f"{q:.6f}" for q in data.qpos) + "\n"
        log_file.write(log_line)
        log_file.flush()

        # Dinamik timestep ayarı
        step_counter += 1
        if step_counter % timestep_update_interval == 0 and auto_timestep:
            # Use PI control to adjust timestep
            target_delta = 0.0  # We want sim_time to match real_time
            error = delta_time - target_delta
            integral_error += error * model.opt.timestep  # Accumulate error
            
            # PI control parameters
            kp = 0.05  # Reduced proportional gain
            ki = 0.01  # Integral gain
            
            # Calculate timestep adjustment using PI control
            p_term = -kp * error
            i_term = -ki * integral_error
            timestep_adjustment = p_term + i_term
            
            # Anti-windup for integral term
            if abs(integral_error) > 10.0:
                integral_error = 10.0 * (integral_error / abs(integral_error))
            
            # Calculate new timestep
            new_timestep = model.opt.timestep + timestep_adjustment
            
            # Apply limits
            model.opt.timestep = min(max(new_timestep, timestep_min), timestep_max)
            # print(f"[AUTO-TIMESTEP] New timestep: {model.opt.timestep:.6f} (delta: {delta_time:.3f}, p_adj: {p_term:.6f}, i_adj: {i_term:.6f})")

        # Render
        width, height = glfw.get_framebuffer_size(window)
        mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjr_render(viewport, scn, con)
        glfw.swap_buffers(window)
        glfw.poll_events()

    stop_flag.set()
    log_file.close()
    glfw.terminate()

if __name__ == "__main__":
    main() 