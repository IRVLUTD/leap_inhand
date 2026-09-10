import socket
import time
import statistics

UDP_IP = "10.42.42.50"
UDP_PORT = 8888
MESSAGE = b"PING"
WARMUP_COUNT = 100
TEST_COUNT = 1000

print(f"Testing UDP connection to OpenRB-150 at {UDP_IP}:{UDP_PORT}")
print("-" * 40)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(2.0) 

try:
    print(f"Running {WARMUP_COUNT} warm-up pings to wake up CPU/Network...")
    # Warm-up phase: do the work but don't record the times
    for _ in range(WARMUP_COUNT):
        sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
        sock.recvfrom(1024)
    
    print(f"Running {TEST_COUNT} measurement pings (Printing disabled for speed)...")
    latencies = []
    
    # We do NOT print inside this loop to avoid terminal overhead 
    # time.perf_counter() provides the highest available resolution clock
    for _ in range(TEST_COUNT):
        start_time = time.perf_counter()
        sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
        sock.recvfrom(1024)
        end_time = time.perf_counter()
        
        latencies.append((end_time - start_time) * 1000)

    # Calculate statistics
    avg_latency = statistics.mean(latencies)
    min_latency = min(latencies)
    max_latency = max(latencies)
    
    # Calculate 95th percentile (95% of packets were faster than this)
    sorted_latencies = sorted(latencies)
    p95_latency = sorted_latencies[int(len(sorted_latencies) * 0.95)]

    print("\n--- Benchmark Results ---")
    print(f"Total Packets: {TEST_COUNT}")
    print(f"Min Latency:   {min_latency:.3f} ms")
    print(f"Avg Latency:   {avg_latency:.3f} ms")
    print(f"95th %ile:     {p95_latency:.3f} ms")
    print(f"Max Latency:   {max_latency:.3f} ms")
    print("-" * 40)

except socket.timeout:
    print("\nERROR: Request timed out! Is the Arduino running?")
except Exception as e:
    print(f"\nAn unexpected error occurred: {e}")
