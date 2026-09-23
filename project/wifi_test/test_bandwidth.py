"""WiFi UDP带宽测试 - 大接收缓冲区"""
import socket, struct, time

ESP32_IP = "192.168.137.74"
UDP_PORT = 5001
DURATION = 60
INTERVAL = 10

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1)
sock.bind(('', 5555))

# 关键：加大接收缓冲区到4MB
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
buf_size = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
print(f"接收缓冲区: {buf_size/1024:.0f} KB")

total_bytes = 0
total_pkts = 0
interval_bytes = 0
interval_pkts = 0
lost = 0
last_seq = -1
peak_bw = 0
started = False
t_start = None
t_interval = None
t_hello = 0

print(f"等待 {ESP32_IP}...\n")

try:
    while True:
        now = time.time()
        if not started or now - t_hello >= 1.0:
            sock.sendto(b"hello", (ESP32_IP, UDP_PORT))
            t_hello = now

        if not started and now - t_hello > DURATION:
            print("超时"); break

        try:
            data, addr = sock.recvfrom(65536)
        except socket.timeout:
            continue

        if len(data) < 4:
            continue

        if not started:
            started = True
            t_start = time.time()
            t_interval = t_start
            print(f"{'时间':>6}s  {'KB/s':>8}  {'峰值':>8}  {'pkt/s':>7}  {'丢包':>6}  {'丢包率':>6}")
            print("-" * 58)

        seq = struct.unpack('<I', data[:4])[0]
        if last_seq >= 0 and seq > last_seq + 1:
            lost += seq - last_seq - 1
        last_seq = seq

        total_bytes += len(data)
        total_pkts += 1
        interval_bytes += len(data)
        interval_pkts += 1

        now = time.time()
        if now - t_start > DURATION:
            break

        if now - t_interval >= INTERVAL:
            elapsed = now - t_start
            dt = now - t_interval
            int_bw = interval_bytes / dt / 1024
            int_pps = interval_pkts / dt
            total_expected = total_pkts + lost
            loss_rate = lost / max(1, total_expected) * 100
            if int_bw > peak_bw:
                peak_bw = int_bw
            print(f"{elapsed:>5.0f}  {int_bw:>8.1f}  {peak_bw:>8.1f}  {int_pps:>7.0f}  {lost:>6}  {loss_rate:>5.1f}%")
            interval_bytes = 0
            interval_pkts = 0
            t_interval = now

except KeyboardInterrupt:
    pass

sock.close()
if started:
    elapsed = time.time() - t_start
    avg_bw = total_bytes / elapsed / 1024
    total_expected = total_pkts + lost
    print("-" * 58)
    print(f"平均: {avg_bw:.1f} KB/s, 峰值: {peak_bw:.1f} KB/s ({peak_bw*8/1024:.2f} Mbps)")
    print(f"收到: {total_pkts} 包, 丢包: {lost} ({lost/max(1,total_expected)*100:.1f}%)")
    print(f"\nJPEG quality=20 (~3KB): {peak_bw/3:.0f} FPS")
    print(f"JPEG quality=50 (~9KB): {peak_bw/9:.0f} FPS")
