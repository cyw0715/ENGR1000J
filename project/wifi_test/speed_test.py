"""
WiFi speed test - PC receiver (fixed rate)
"""
import socket
import time

HOST = '0.0.0.0'
PORT = 5000

def main():
    print(f"Listening on {HOST}:{PORT}...")
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)

    conn, addr = server.accept()
    print(f"Connected: {addr}")

    total = 0
    interval_bytes = 0
    t0 = time.time()
    last_report = t0

    try:
        while True:
            data = conn.recv(65536)
            if not data:
                break
            total += len(data)
            interval_bytes += len(data)

            now = time.time()
            if now - last_report >= 1:
                dt = now - last_report
                rate = interval_bytes / dt / 1024  # KB/s (瞬时速率)
                print(f"[{now - t0:.0f}s] total={total/1024:.0f}KB rate={rate:.1f} KB/s")
                interval_bytes = 0
                last_report = now

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - t0
        avg = total / elapsed / 1024
        print(f"\nTotal: {total/1024:.0f} KB in {elapsed:.1f}s, avg: {avg:.1f} KB/s")
        conn.close()
        server.close()

if __name__ == '__main__':
    main()
