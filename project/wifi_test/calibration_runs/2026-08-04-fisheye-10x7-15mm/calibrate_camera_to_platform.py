"""Automatic camera-to-board pose capture using accepted fisheye intrinsics.

Physical convention for this run:
- Board is flat under the camera.
- 200 mm page long edge is parallel to platform left/right.
- The PDF text/bottom margin points toward platform FRONT.
No actuator command or HTTP write is performed.
"""
from __future__ import annotations
import argparse, concurrent.futures, json, math, socket, struct, time, urllib.request
from datetime import datetime, timezone
from pathlib import Path
import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
CAL_PATH = ROOT / "fisheye_calibration.json"
OUT_JSON = ROOT / "camera_to_board_pose.json"
OUT_IMAGE = ROOT / "camera_to_board_axes.jpg"
PATTERN = (10, 7)
SQUARE_MM = 15.0
REQUIRED_STABLE = 20


def discover_p4():
    def probe(host):
        try:
            with urllib.request.urlopen(f"http://{host}/board", timeout=.45) as r:
                d=json.load(r)
            return host if d.get("board")=="ESP32-P4" and d.get("ip")==host else None
        except Exception: return None
    hosts=[f"192.168.137.{i}" for i in range(1,255)]
    with concurrent.futures.ThreadPoolExecutor(max_workers=48) as pool:
        found=[x for x in pool.map(probe,hosts) if x]
    if not found: raise RuntimeError("no P4 found")
    return found[0]


def recv_exact(s,n):
    out=bytearray()
    while len(out)<n:
        chunk=s.recv(n-len(out))
        if not chunk: raise ConnectionError("video closed")
        out.extend(chunk)
    return bytes(out)


def frame(s):
    n=struct.unpack("<I",recv_exact(s,4))[0]
    if not 1000<=n<=300000: raise ValueError(f"bad JPEG size {n}")
    data=recv_exact(s,n)
    x=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_GRAYSCALE)
    if x is None: raise ValueError("decode failed")
    return x


def detect(x):
    flags=cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE
    ok,c=cv2.findChessboardCorners(x,PATTERN,flags)
    if not ok:
        ok,c=cv2.findChessboardCornersSB(x,PATTERN,flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:return None
    return cv2.cornerSubPix(x,c.astype(np.float32),(7,7),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,50,1e-4)).reshape(1,-1,2)


def solve_pose(corners,K,D):
    obj=np.zeros((PATTERN[0]*PATTERN[1],3),np.float64)
    obj[:,:2]=np.mgrid[0:PATTERN[0],0:PATTERN[1]].T.reshape(-1,2)*SQUARE_MM
    norm=cv2.fisheye.undistortPoints(corners.astype(np.float64),K,D).reshape(-1,1,2)
    ok,rvec,tvec=cv2.solvePnP(obj,norm,np.eye(3),None,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok: raise RuntimeError("solvePnP failed")
    projected,_=cv2.fisheye.projectPoints(obj.reshape(1,-1,3),rvec,tvec,K,D)
    error=float(np.sqrt(np.mean(np.sum((projected.reshape(-1,2)-corners.reshape(-1,2))**2,axis=1))))
    R,_=cv2.Rodrigues(rvec)
    C=-R.T@tvec
    optical=R.T@np.array([[0.],[0.],[1.]])
    tilt=math.degrees(math.acos(np.clip(abs(float(optical[2,0]))/np.linalg.norm(optical),-1,1)))
    return {"rvec":rvec.reshape(3),"tvec":tvec.reshape(3),"R":R,"C":C.reshape(3),"optical":optical.reshape(3),"error":error,"tilt":tilt}


def robust_result(samples):
    errors=np.array([s["error"] for s in samples])
    heights=np.array([abs(s["C"][2]) for s in samples])
    med=np.median(heights); mad=np.median(np.abs(heights-med))
    keep=(errors<1.5)&(np.abs(heights-med)<=max(3*mad,3.0))
    chosen=[s for s,k in zip(samples,keep) if k]
    if len(chosen)<12: raise RuntimeError(f"insufficient stable pose samples: {len(chosen)}")
    rv=np.median(np.array([s["rvec"] for s in chosen]),axis=0).reshape(3,1)
    tv=np.median(np.array([s["tvec"] for s in chosen]),axis=0).reshape(3,1)
    R,_=cv2.Rodrigues(rv); C=(-R.T@tv).reshape(3); optical=(R.T@np.array([[0.],[0.],[1.]])).reshape(3)
    tilt=math.degrees(math.acos(np.clip(abs(optical[2])/np.linalg.norm(optical),-1,1)))
    return chosen,rv,tv,R,C,optical,tilt


def main():
    ap=argparse.ArgumentParser(); ap.add_argument("ip",nargs="?"); args=ap.parse_args()
    cal=json.loads(CAL_PATH.read_text(encoding="utf-8"))
    if not cal.get("accepted_for_geometry"): raise RuntimeError("intrinsics not accepted")
    K=np.array(cal["K"],np.float64); D=np.array(cal["D"],np.float64).reshape(4,1)
    ip=args.ip or discover_p4(); s=socket.create_connection((ip,5000),5); s.settimeout(8)
    samples=[]; previous=None; last=None
    print(f"CONNECTED {ip}:5000; waiting for flat 10x7 board",flush=True)
    try:
        while True:
            x=frame(s); c=detect(x); vis=cv2.cvtColor(x,cv2.COLOR_GRAY2BGR)
            status="NO 10x7 BOARD"; color=(0,0,255)
            if c is not None:
                cv2.drawChessboardCorners(vis,PATTERN,c,True)
                motion=float("inf") if previous is None else float(np.mean(np.linalg.norm(c.reshape(-1,2)-previous.reshape(-1,2),axis=1)))
                previous=c.copy(); pose=solve_pose(c,K,D); last=(x,c,pose)
                if motion<=1.5 and pose["error"]<1.5:
                    samples.append(pose)
                else: samples.clear()
                status=f"HOLD STILL {len(samples)}/{REQUIRED_STABLE} motion={motion:.2f}px reproj={pose['error']:.2f}px"
                color=(0,220,255)
                if len(samples)>=REQUIRED_STABLE:
                    break
            cv2.putText(vis,status,(10,28),cv2.FONT_HERSHEY_SIMPLEX,.55,color,2,cv2.LINE_AA)
            cv2.putText(vis,"Paper text edge -> platform FRONT",(10,54),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),2,cv2.LINE_AA)
            cv2.imshow("Camera to Platform Extrinsics",vis)
            if cv2.waitKey(1)&0xff in (27,ord('q')): raise KeyboardInterrupt
    finally:
        s.close(); cv2.destroyAllWindows()
    chosen,rv,tv,R,C,optical,tilt=robust_result(samples)
    x,c,_=last; overlay=cv2.cvtColor(x,cv2.COLOR_GRAY2BGR); pts=c.reshape(PATTERN[1],PATTERN[0],2)
    origin=tuple(np.round(pts[0,0]).astype(int)); xp=tuple(np.round(pts[0,-1]).astype(int)); yp=tuple(np.round(pts[-1,0]).astype(int))
    cv2.arrowedLine(overlay,origin,xp,(0,0,255),4,cv2.LINE_AA,tipLength=.08)
    cv2.arrowedLine(overlay,origin,yp,(0,255,0),4,cv2.LINE_AA,tipLength=.10)
    cv2.putText(overlay,"board +X RED",xp,cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,255),2,cv2.LINE_AA)
    cv2.putText(overlay,"board +Y GREEN",yp,cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,0),2,cv2.LINE_AA)
    cv2.imwrite(str(OUT_IMAGE),overlay,[cv2.IMWRITE_JPEG_QUALITY,95])
    report={
      "schema":"mars-lander-camera-to-board-v1","created_utc":datetime.now(timezone.utc).isoformat(),
      "intrinsics_path":str(CAL_PATH),"image_size":cal["image_size"],"samples":len(chosen),
      "board":{"inner_corners":list(PATTERN),"square_mm":SQUARE_MM,
        "placement":"flat; long edge platform left-right; PDF text edge points platform front"},
      "R_board_to_camera":R.tolist(),"t_board_to_camera_mm":tv.reshape(3).tolist(),
      "camera_center_board_mm":C.tolist(),"camera_height_above_board_mm":float(abs(C[2])),
      "camera_optical_axis_in_board":optical.tolist(),"optical_axis_tilt_from_board_normal_deg":float(tilt),
      "sample_reprojection_mean_px":float(np.mean([q['error'] for q in chosen])),
      "sample_reprojection_max_px":float(max(q['error'] for q in chosen)),
      "axis_mapping_validated":False,
      "warning":"Checkerboard ordering has a possible 180-degree platform-axis ambiguity; validate saved red/green arrows before control.",
      "overlay_path":str(OUT_IMAGE)
    }
    OUT_JSON.write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding="utf-8")
    print(json.dumps(report,indent=2,ensure_ascii=False),flush=True)

if __name__=="__main__": main()
