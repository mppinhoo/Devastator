#!/usr/bin/env python3
import psutil
import subprocess
import time
import csv
import threading
import datetime

class SLAMMonitor:
    def __init__(self, duration=300, interval=1, output_file="slam_metrics.csv"):
        # Initialize variables
        self.duration = duration
        self.interval = interval
        self.output_file = output_file
        self.start_time = time.time()

        # RAM buffer for storing data before writing to CSV
        self.data_buffer = []

        # Open CSV file for writing
        self.csv_file = open(self.output_file, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        # Write header
        self.csv_writer.writerow([
            "Time", "CPU Percent", "Memory Used (MB)", "Memory Percent", 
            "Absolute CPU (s)"
        ])

        # Get the SLAM Toolbox PID
        self.pid = self.get_slam_toolbox_pid()
        if not self.pid:
            print("SLAM Toolbox is not running.")
            return

        # Start a background timer to flush data every 1 second
        self.flush_timer = threading.Timer(self.interval, self.flush_to_csv)
        self.flush_timer.start()

        # Start the monitoring process
        self.monitor_process()

    def get_slam_toolbox_pid(self):
        """Fetch the PID of the SLAM Toolbox process."""
        try:
            output = subprocess.check_output(["pgrep", "-f", "slam_toolbox"])
            return int(output.splitlines()[0])
        except subprocess.CalledProcessError:
            return None

    def monitor_process(self):
        """Monitor CPU and memory usage of the SLAM Toolbox process."""
        # Initial absolute CPU time
        process = psutil.Process(self.pid)
        initial_cpu_times = process.cpu_times()

        # Main monitoring loop
        while time.time() - self.start_time < self.duration:
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
                current_time = datetime.datetime.now().strftime('%H:%M:%S')

                # Add the current data to the buffer
                self.data_buffer.append([
                    current_time, cpu_percent, memory_used_mb, memory_percent, absolute_cpu_time
                ])

                # Update the initial_cpu_times for the next loop iteration
                initial_cpu_times = current_cpu_times

            except psutil.NoSuchProcess:
                print("SLAM Toolbox process terminated.")
                break
            except psutil.AccessDenied:
                print("Access denied to the process.")
                break

            # Wait for the next interval while keeping timing consistent
            time_to_sleep = self.interval - (time.time() - loop_start)
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)

    def flush_to_csv(self):
        """Flushes stored data to CSV every second."""
        if self.data_buffer:
            self.csv_writer.writerows(self.data_buffer)
            self.csv_file.flush()
            self.data_buffer = []  # Clear buffer after writing

        # Restart the timer
        self.flush_timer = threading.Timer(self.interval, self.flush_to_csv)
        self.flush_timer.start()

    def stop_monitoring(self):
        """Stop the monitoring process and ensure data is saved."""
        if self.flush_timer.is_alive():
            self.flush_timer.cancel()

        self.flush_to_csv()  # Final flush
        self.csv_file.close()

if __name__ == "__main__":
    monitor = SLAMMonitor(duration=300, interval=1, output_file="slam_metrics.csv")
    try:
        # Wait until the monitoring duration is over
        time.sleep(monitor.duration)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.stop_monitoring()
