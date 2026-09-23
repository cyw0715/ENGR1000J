"""
Network scanner - scan 192.168.137.x for active hosts
"""
import socket
import concurrent.futures

def ping_host(ip):
    """Try to connect to common ports"""
    for port in [80, 443, 5000, 22, 8080]:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(0.3)
            result = sock.connect_ex((ip, port))
            sock.close()
            if result == 0:
                return ip, port
        except:
            pass
    return None

def main():
    base_ip = "192.168.137."
    print(f"Scanning {base_ip}0/24...")

    ips = [base_ip + str(i) for i in range(1, 255)]
    found = []

    with concurrent.futures.ThreadPoolExecutor(max_workers=50) as executor:
        futures = {executor.submit(ping_host, ip): ip for ip in ips}
        for future in concurrent.futures.as_completed(futures):
            result = future.result()
            if result:
                ip, port = result
                print(f"  Found: {ip}:{port}")
                found.append((ip, port))

    print(f"\nDone. Found {len(found)} hosts.")

if __name__ == '__main__':
    main()
