"""Mars Lander - OV5647 chessboard intrinsic calibration (TCP JPEG).

Board specification: 9 columns x 6 rows of squares, each 20.0 mm.
OpenCV uses the number of INNER corners: 8 columns x 5 rows.
"""
import socket, struct, time, os, sys, numpy as np, cv2

PROJECT_ROOT = os.path.dirname(os.path.dirname(__file__))
PC_VIEWER_DIR = os.path.join(PROJECT_ROOT, "pc_viewer")
if PC_VIEWER_DIR not in sys.path:
    sys.path.insert(0, PC_VIEWER_DIR)
from esp_discovery import discover_esp32

PC_PORT = 5000
SAVE_DIR = os.path.join(os.path.dirname(__file__), "calib_frames")
PATTERN = (8, 5)  # inner corners: 9x6 squares -> 8x5 corners
SQUARE_MM = 20.0
MIN_PHOTOS = 15


def receive_one_frame(sock, timeout=5):
    sock.settimeout(timeout)
    try:
        hdr = b''
        while len(hdr) < 4:
            chunk = sock.recv(4 - len(hdr))
            if not chunk: return None
            hdr += chunk
        size = struct.unpack('<I', hdr)[0]
        if size < 1000 or size > 200000: return None
        buf = b''
        while len(buf) < size:
            chunk = sock.recv(min(65536, size - len(buf)))
            if not chunk: break
            buf += chunk
        if len(buf) < size: return None
        return cv2.imdecode(np.frombuffer(buf, np.uint8), cv2.IMREAD_GRAYSCALE)
    except: return None


def run_calibration(image_dir):
    print(f"\n{'='*50}\n  标定 ({PATTERN[0]}x{PATTERN[1]}, {SQUARE_MM}mm)\n{'='*50}\n")
    objp = np.zeros((PATTERN[0]*PATTERN[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:PATTERN[0], 0:PATTERN[1]].T.reshape(-1, 2) * SQUARE_MM
    obj_points, img_points, img_size = [], [], None
    files = sorted(f for f in os.listdir(image_dir) if f.endswith('.jpg'))
    if not files: print("没有截图！"); return None
    for fname in files:
        img = cv2.imread(os.path.join(image_dir, fname), cv2.IMREAD_GRAYSCALE)
        if img is None: continue
        if img_size is None: img_size = img.shape[::-1]
        ret, corners = cv2.findChessboardCorners(img, PATTERN, None)
        if ret:
            corners2 = cv2.cornerSubPix(img, corners, (11,11), (-1,-1), (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            obj_points.append(objp); img_points.append(corners2)
            print(f"  {fname}: OK")
        else: print(f"  {fname}: NO")
    if len(obj_points) < MIN_PHOTOS: print(f"\n需要 {MIN_PHOTOS}，当前 {len(obj_points)}"); return None
    print(f"\n标定中...")
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, img_size, None, None)
    per_view_errors = []
    for i in range(len(obj_points)):
        projected, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        observed_xy = img_points[i].reshape(-1, 2)
        projected_xy = projected.reshape(-1, 2).astype(observed_xy.dtype)
        per_view_errors.append(cv2.norm(observed_xy, projected_xy, cv2.NORM_L2) / len(observed_xy))
    err = float(np.mean(per_view_errors))
    print("  每张重投影误差(px): " + ", ".join(f"{value:.3f}" for value in per_view_errors))
    print(f"\n{'='*50}\n  结果\n{'='*50}")
    print(f"  fx={K[0,0]:.2f} fy={K[1,1]:.2f} cx={K[0,2]:.2f} cy={K[1,2]:.2f}")
    print(f"  重投影误差={err:.4f} ({'合格' if err<1 else '偏大'})")
    print(f"  畸变={dist.ravel()}\n{'='*50}")
    cfg = os.path.join(os.path.dirname(image_dir), "camera_config.py")
    with open(cfg, 'w') as f:
        f.write(f'"""OV5647标定"""\nimport numpy as np\nCAMERA_MATRIX=np.array([[{K[0,0]:.6f},{K[0,1]:.6f},{K[0,2]:.6f}],[{K[1,0]:.6f},{K[1,1]:.6f},{K[1,2]:.6f}],[{K[2,0]:.6f},{K[2,1]:.6f},{K[2,2]:.6f}]])\nDIST_COEFFS=np.array({dist.ravel().tolist()})\nFX={K[0,0]:.2f}\nFY={K[1,1]:.2f}\nCX={K[0,2]:.2f}\nCY={K[1,2]:.2f}\n')
    print(f"保存: {cfg}")
    return K, dist


def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    esp_ip = sys.argv[1] if len(sys.argv) > 1 else discover_esp32()
    print(f"棋盘规格: 9x6 方格 / {PATTERN[0]}x{PATTERN[1]} 内角点 / {SQUARE_MM:.1f} mm")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(10)
    print(f"连接 {esp_ip}:{PC_PORT}...")
    try:
        sock.connect((esp_ip, PC_PORT))
        print("已连接！\n")
    except Exception as e:
        print(f"连接失败: {e}"); return

    print("操作: 'a'=自动 's'=截图 'c'=标定 'd'=删除 'q'=退出\n")
    saved = sum(1 for f in os.listdir(SAVE_DIR) if f.endswith('.jpg'))
    auto = True; last_t = 0; positions = []

    def corner_center(corners):
        points = corners.reshape(-1, 2)
        return float(np.mean(points[:, 0])), float(np.mean(points[:, 1]))

    def is_new(corners):
        cx, cy = corner_center(corners)
        return all(np.sqrt((cx-px)**2+(cy-py)**2) >= 80 for px,py in positions)

    cv2.namedWindow('Calibration', cv2.WINDOW_NORMAL)
    try:
        while True:
            img = receive_one_frame(sock)
            if img is None: print("断开"); break

            disp = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            ret, corners = cv2.findChessboardCorners(img, PATTERN, None)
            now = time.time()
            if ret:
                cv2.drawChessboardCorners(disp, PATTERN, corners, ret)
                cv2.putText(disp, "DETECTED", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                if auto and now-last_t > 2.0 and is_new(corners):
                    saved += 1
                    cv2.imwrite(os.path.join(SAVE_DIR, f"calib_{saved:03d}.jpg"), img)
                    positions.append(corner_center(corners))
                    last_t = now; print(f"  截图 #{saved}")
            else:
                cv2.putText(disp, "No chessboard", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            cv2.putText(disp, f"[{'AUTO' if auto else 'MANUAL'}] {saved}/{MIN_PHOTOS}", (10,95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
            cv2.imshow('Calibration', disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            elif key == ord('a'): auto = not auto
            elif key == ord('s') and ret:
                saved += 1
                cv2.imwrite(os.path.join(SAVE_DIR, f"calib_{saved:03d}.jpg"), img)
                positions.append(corner_center(corners))
                print(f"  截图 #{saved}")
            elif key == ord('d'):
                fs = sorted(f for f in os.listdir(SAVE_DIR) if f.endswith('.jpg'))
                if fs: os.remove(os.path.join(SAVE_DIR, fs[-1])); saved -= 1; positions.pop() if positions else None; print(f"  删除，剩余 {saved}")
            elif key == ord('c'):
                if saved < MIN_PHOTOS: print(f"  需要 {MIN_PHOTOS}，当前 {saved}")
                else: sock.close(); cv2.destroyAllWindows(); run_calibration(SAVE_DIR); return
    except KeyboardInterrupt: pass
    finally: sock.close(); cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
