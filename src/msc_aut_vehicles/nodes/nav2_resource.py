#!/usr/bin/env python3

import psutil
import subprocess
import time
import csv


def get_pid_by_name(target_name):
    """Get PID for the first process matching the target_name."""
    for proc in psutil.process_iter(attrs=['pid', 'name', 'cmdline']):
        try:
            if target_name in ' '.join(proc.info['cmdline']):
                return proc.info['pid']
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return None


def get_nav2_pids():
    """Get all PIDs of processes containing 'nav2_' in their command line."""
    nav2_pids = []
    for proc in psutil.process_iter(attrs=['pid', 'cmdline']):
        try:
            cmdline = ' '.join(proc.info['cmdline'])
            if 'nav2_' in cmdline:
                nav2_pids.append(proc.info['pid'])
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return nav2_pids


def monitor_nav2_resources(duration=300, interval=1, output_file="nav2_controller_metrics.csv"):
    controller_pid = get_pid_by_name("controller_server")
    nav2_pids = get_nav2_pids()

    if controller_pid is None:
        print("controller_server process not found.")
        return

    if not nav2_pids:
        print("No nav2_* processes found.")
        return

    #print(f"Tracking controller_server (PID {controller_pid})")
    #print(f"Tracking nav2 processes: {nav2_pids}")

    controller_proc = psutil.Process(controller_pid)
    nav2_procs = [psutil.Process(pid) for pid in nav2_pids]

    start_time = time.time()

    with open(output_file, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            "Time",
            # Controller server
            "Controller CPU (%)", "Controller Memory (MB)", "Controller Mem (%)", "Controller Abs CPU (s)",
            # Total nav2
            "Nav2 CPU (%)", "Nav2 Memory (MB)", "Nav2 Mem (%)", "Nav2 Abs CPU (s)"
        ])

        # Initial CPU times
        initial_ctrl_cpu = controller_proc.cpu_times()
        initial_nav2_cpu = {p.pid: p.cpu_times() for p in nav2_procs}

        while time.time() - start_time < duration:
            loop_start = time.time()
            try:
                # --- Controller server ---
                ctrl_cpu = controller_proc.cpu_percent(interval=0.0)
                ctrl_mem = controller_proc.memory_info()
                ctrl_mem_mb = ctrl_mem.rss / (1024 * 1024)
                ctrl_mem_percent = controller_proc.memory_percent()
                ctrl_curr_cpu = controller_proc.cpu_times()
                ctrl_abs_cpu = (ctrl_curr_cpu.user + ctrl_curr_cpu.system) - \
                               (initial_ctrl_cpu.user + initial_ctrl_cpu.system)
                initial_ctrl_cpu = ctrl_curr_cpu

                # --- Nav2 total ---
                nav2_cpu = 0.0
                nav2_mem_mb = 0.0
                nav2_mem_percent = 0.0
                nav2_abs_cpu = 0.0

                for p in nav2_procs:
                    try:
                        nav2_cpu += p.cpu_percent(interval=0.0)
                        nav2_mem_mb += p.memory_info().rss / (1024 * 1024)
                        nav2_mem_percent += p.memory_percent()
                        curr = p.cpu_times()
                        init = initial_nav2_cpu[p.pid]
                        nav2_abs_cpu += (curr.user + curr.system) - (init.user + init.system)
                        initial_nav2_cpu[p.pid] = curr
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue

                current_time = time.strftime('%H:%M:%S')
                writer.writerow([
                    current_time,
                    ctrl_cpu, ctrl_mem_mb, ctrl_mem_percent, ctrl_abs_cpu,
                    nav2_cpu, nav2_mem_mb, nav2_mem_percent, nav2_abs_cpu
                ])

            except psutil.NoSuchProcess:
                print("A process ended prematurely.")
                break

            # Maintain timing
            time_to_sleep = interval - (time.time() - loop_start)
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)

    print(f"Logging complete. Data saved to {output_file}")


if __name__ == "__main__":
    monitor_nav2_resources(duration=300, interval=1)
