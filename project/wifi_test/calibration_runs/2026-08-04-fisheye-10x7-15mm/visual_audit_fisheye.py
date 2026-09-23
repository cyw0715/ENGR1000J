from __future__ import annotations
import json
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent
CAL=json.loads((ROOT/'fisheye_calibration.json').read_text(encoding='utf-8'))
K=np.array(CAL['K'],np.float64); D=np.array(CAL['D'],np.float64).reshape(4,1)
SIZE=tuple(CAL['image_size'])
PICKS=['view_034.jpg','view_036.jpg','view_014.jpg','view_005.jpg','view_011.jpg']
OUT=ROOT/'audit'; OUT.mkdir(exist_ok=True)

# Undistortion audit for current frame and extreme calibration views.
for name in ['current_frame.jpg',*PICKS]:
    src=cv2.imread(str(ROOT/name if name=='current_frame.jpg' else ROOT/'views'/name),cv2.IMREAD_GRAYSCALE)
    if src is None: continue
    panels=[cv2.cvtColor(src,cv2.COLOR_GRAY2BGR)]
    cv2.putText(panels[0],'DISTORTED',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2,cv2.LINE_AA)
    for balance in (0.0,0.5,1.0):
        newK=cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K,D,SIZE,np.eye(3),balance=balance,new_size=SIZE,fov_scale=1.0)
        map1,map2=cv2.fisheye.initUndistortRectifyMap(K,D,np.eye(3),newK,SIZE,cv2.CV_16SC2)
        und=cv2.remap(src,map1,map2,cv2.INTER_LANCZOS4,borderMode=cv2.BORDER_CONSTANT)
        panel=cv2.cvtColor(und,cv2.COLOR_GRAY2BGR)
        cv2.putText(panel,f'UNDISTORT balance={balance:.1f}',(12,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,0),2,cv2.LINE_AA)
        panels.append(panel)
    montage=np.hstack(panels)
    cv2.imwrite(str(OUT/f'{Path(name).stem}_undistort_montage.jpg'),montage,[cv2.IMWRITE_JPEG_QUALITY,95])

# Reprojection overlays use saved per-view extrinsics recomputed from calibrated K/D.
obj=np.zeros((1,70,3),np.float64); obj[0,:,:2]=np.mgrid[0:10,0:7].T.reshape(-1,2)*15.0
for name in PICKS:
    src=cv2.imread(str(ROOT/'views'/name),cv2.IMREAD_GRAYSCALE)
    found,corners=cv2.findChessboardCorners(src,(10,7),cv2.CALIB_CB_ADAPTIVE_THRESH|cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found: continue
    corners=cv2.cornerSubPix(src,corners,(7,7),(-1,-1),(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,50,1e-4)).reshape(1,-1,2).astype(np.float64)
    ok,rvec,tvec=cv2.solvePnP(obj.reshape(-1,3),corners.reshape(-1,2),K,D,flags=cv2.SOLVEPNP_ITERATIVE)
    # solvePnP is pinhole and not valid for fisheye; undistort points then solve normalized.
    norm=cv2.fisheye.undistortPoints(corners,K,D).reshape(-1,2)
    ok,rvec,tvec=cv2.solvePnP(obj.reshape(-1,3),norm,np.eye(3),None,flags=cv2.SOLVEPNP_ITERATIVE)
    projected,_=cv2.fisheye.projectPoints(obj,rvec,tvec,K,D)
    vis=cv2.cvtColor(src,cv2.COLOR_GRAY2BGR)
    for observed,predicted in zip(corners.reshape(-1,2),projected.reshape(-1,2)):
        cv2.circle(vis,tuple(np.round(observed).astype(int)),3,(0,255,0),-1,cv2.LINE_AA)
        cv2.drawMarker(vis,tuple(np.round(predicted).astype(int)),(0,0,255),cv2.MARKER_CROSS,8,1,cv2.LINE_AA)
    err=np.sqrt(np.mean(np.sum((corners.reshape(-1,2)-projected.reshape(-1,2))**2,axis=1)))
    cv2.putText(vis,f'green=observed red=projected RMS={err:.3f}px',(10,28),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,0),2,cv2.LINE_AA)
    cv2.imwrite(str(OUT/f'{Path(name).stem}_reprojection.jpg'),vis,[cv2.IMWRITE_JPEG_QUALITY,95])

print({'K':K.tolist(),'D':D.reshape(-1).tolist(),'output':str(OUT),'files':len(list(OUT.glob('*.jpg')))})
