#!/usr/bin/env python3
import psutil
import subprocess
import time
import csv

def get_slam_toolbox_pid():
    try:
        output = subprocess.check_output(["pgrep", "-f", "slam_toolbox"])
        return int(output.splitlines()[0])
    except subprocess.CalledProcessError:
        return None

def monitor_process(pid, duration=300, interval=1, output_file="slam_metrics.csv"):
    # Initialize variables
    start_time = time.time()

    # Initial absolute CPU time
    process = psutil.Process(pid)
    initial_cpu_times = process.cpu_times()

    # Open CSV file for writing
    with open(output_file, mode="w", newline="") as file:
        writer = csv.writer(file)
        # Write header
        writer.writerow([
            "Time", "CPU Percent", "Memory Used (MB)", "Memory Percent", 
            "Absolute CPU (s)"
        ])

        while time.time() - start_time < duration:
            loop_start = time.time()
            try:
                cpu_percent = process.cpu_percent(interval=0.1)
                memory_info = process.memory_info()
                memory_used_mb = memory_info.rss / (1024 * 1024)  # Convert to MB
                memory_percent = process.memory_percent()

                # Calculate absolute CPU time
                current_cpu_times = process.cpu_times()
                absolute_cpu_time = (current_cpu_times.user + current_cpu_times.system) - (initial_cpu_times.user + initial_cpu_times.system)

                # Get the current time
                current_time = time.strftime('%H:%M:%S')

                # Write the current data to CSV file
                writer.writerow([
                    current_time, cpu_percent, memory_used_mb, memory_percent, 
                    absolute_cpu_time
                ])

                # Print current and average resource usage
                #print(f"[{current_time}] CPU: {cpu_percent:.2f}% | Memory: {memory_used_mb:.2f} MB ({memory_percent:.2f}%) "
                #     f"Absolute CPU: {absolute_cpu_time:.2f} s | Averages -> CPU: {avg_cpu:.2f}% | "
                #      f"Memory: {avg_memory_mb:.2f} MB ({avg_memory_percent:.2f}%) | Absolute CPU: {avg_absolute_cpu:.2f} s")

                # Update the initial_cpu_times for the next loop iteration
                initial_cpu_times = current_cpu_times

            except psutil.NoSuchProcess:
                #print("SLAM Toolbox process terminated.")
                break
            except psutil.AccessDenied:
                #print("Access denied to the process.")
                break

            # Wait for the next interval while keeping timing consistent
            time_to_sleep = interval - (time.time() - loop_start)
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)

if __name__ == "__main__":
    #print("Fetching SLAM Toolbox PID...")
    pid = get_slam_toolbox_pid()
    if pid:
        monitor_process(pid, duration=300, interval=1, output_file="slam_metrics.csv")
    else:
        print("SLAM Toolbox is not running.")
