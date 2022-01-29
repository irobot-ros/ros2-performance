#!/usr/bin/env python3

import argparse
import csv
import psutil
import signal
import statistics
import time

def process_is_alive(p):
  return p.status() not in [psutil.STATUS_ZOMBIE, psutil.STATUS_DEAD]

def b_to_mb(b):
  # Converts Byte into MegaByte
  return b / (1024 * 1024)

class TotalStats:
  def  __init__(self):
    self.cpu_samples = []

  def add_sample(self, cpu):
    self.cpu_samples.append(cpu)

  def get_avg_cpu(self):
    if not self.cpu_samples:
      return 0.0
    else:
      return statistics.mean(self.cpu_samples)

def compute_stats(processes, file_path, t):
  vms_list = []
  max_vms = 0
  pss_mb = 0
  rss_mb = 0
  uss_mb = 0
  cpu_pct = 0
  for p in processes:
    if not process_is_alive(p):
      continue
    cpu_pct += p.cpu_percent()

    mem_info = p.memory_full_info()
    this_rss_mb = b_to_mb(mem_info.rss)
    this_uss_mb = b_to_mb(mem_info.uss)
    this_pss_mb = b_to_mb(mem_info.pss)
    this_vms_mb = b_to_mb(mem_info.vms)

    vms_list.append(this_vms_mb)
    pss_mb += this_pss_mb
    rss_mb += this_rss_mb
    uss_mb += this_uss_mb

  if vms_list:
    max_vms = max(vms_list)
  else:
    print("WARNING! empty vms list")

  if file_path:
    with open(file_path, 'a') as file:
      tsv_writer = csv.writer(file, delimiter='\t')
      tsv_writer.writerow([t, cpu_pct, round(pss_mb, 4), round(rss_mb, 4), round(max_vms, 4)])
  else:
    print(f"-T {t} - CPU[%] {cpu_pct} PSS[MB] {round(pss_mb, 4)} RSS[MB] {round(rss_mb, 4)} VMS[MB] {round(max_vms, 4)}")

  return cpu_pct

def start_processes(procs):
  processes = []
  for proc_input in procs:
    p = psutil.Popen(
      proc_input,
      cwd=None,
      stdout=None,
      stderr=None)

    print(f"Started {proc_input} in {p.pid}")
    processes.append(p)
  
  return processes

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument(
    '-p', '--process', action='append', nargs='+', default=[],
    help='Command to start a process that will be monitored. You can pass -p more than once.')
  parser.add_argument(
    '--step', type=float, default=0.2,
    help='Interval of time between performance metrics computation in seconds')
  parser.add_argument(
    '-t', '--duration', type=float, default=5.0,
    help='Duration of the test in seconds.')
  parser.add_argument(
    '--file', type=str, default=None,
    help='Filepath where to store the logs. Will print to screen if nothing is provided')

  args = parser.parse_args()

  if not args.process:
    print("ERROR! You must provide at least one process to monitor!")
    assert(False)

  if args.file is not None:
    print(f"Logging statistics to {args.file}")
    with open(args.file, 'w') as file:
      tsv_writer = csv.writer(file, delimiter='\t')
      tsv_writer.writerow(['Time[sec]', 'CPU[%]', 'PSS[MB]', 'RSS[MB]', 'MaxVMS[MB]'])

  # Start all processes
  processes = start_processes(args.process)

  time.sleep(0.5)
  # Compute performance metrics while the processes run
  start_time = time.time()
  current_time = time.time() - start_time
  total_stats = TotalStats()
  while any(process_is_alive(proc) for proc in processes) and current_time < args.duration:
    cpu_pct = compute_stats(processes, args.file, round(current_time, 2))
    total_stats.add_sample(cpu_pct)
    time.sleep(args.step)
    current_time = time.time() - start_time

  print(f"CPU MEAN: {total_stats.get_avg_cpu()}")

  # After the test is done, kill all processes that we started
  # We use SIGINT to have a graceful shutdown, ROS does not catch SIGTERM or SIGKILL
  for p in processes:
    print(f"Sending SIGINT to {p.status()} process {p.pid}")
    p.send_signal(signal.SIGINT)
    p.wait()

if __name__ == '__main__':
  main()
