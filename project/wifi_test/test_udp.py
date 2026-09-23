"""UDP接收调试"""
import socket, struct
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 5555))
sock.settimeout(3)
sock.sendto(b"hello", ("192.168.137.74", 5001))
print("等待数据...")
for i in range(20):
    try:
        data, addr = sock.recvfrom(65536)
        print(f"pkt {i}: {len(data)}B from {addr}")
        if len(data) >= 12:
            seq=struct.unpack('<I',data[:4])[0]
            cid=struct.unpack('<H',data[4:6])[0]
            tot=struct.unpack('<H',data[6:8])[0]
            sz=struct.unpack('<I',data[8:12])[0]
            print(f"  seq={seq} cid={cid:#x} total={tot} size={sz}")
    except socket.timeout:
        print("timeout"); break
sock.close()
