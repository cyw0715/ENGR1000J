import cv2, glob, importlib.util, numpy as np
p=r'E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\wifi_test\calibration_runs\2026-08-04-fisheye-10x7-15mm\capture_and_calibrate_fisheye.py'
s=importlib.util.spec_from_file_location('cal',p); m=importlib.util.module_from_spec(s); s.loader.exec_module(m)
imgs=[]
for f in glob.glob(r'E:\Chenyuewei\University\Freshman\summer\ENGR1000\p2\project\wifi_test\calibration_runs\2026-08-04-fisheye-10x7-15mm\views\*.jpg'):
 x=cv2.imread(f,0); c=m.detect(x); imgs.append(c.astype(np.float64))
base=np.zeros((m.PATTERN[0]*m.PATTERN[1],3),np.float64); base[:,:2]=np.mgrid[0:m.PATTERN[0],0:m.PATTERN[1]].T.reshape(-1,2)*m.SQUARE_MM
for objshape in ('N13','1N3'):
 for imgshape in ('N12','1N2'):
  for useguess in (False,True):
   objs=[base.reshape((-1,1,3) if objshape=='N13' else (1,-1,3)).copy() for _ in imgs]
   ips=[x.reshape((-1,1,2) if imgshape=='N12' else (1,-1,2)).copy() for x in imgs]
   K=np.array([[400.,0,400.],[0,400.,400.],[0,0,1]],np.float64); D=np.zeros((4,1),np.float64)
   flags=cv2.CALIB_RECOMPUTE_EXTRINSIC|cv2.CALIB_CHECK_COND|cv2.CALIB_FIX_SKEW|(cv2.CALIB_USE_INTRINSIC_GUESS if useguess else 0)
   try:
    r=cv2.fisheye.calibrate(objs,ips,(800,800),K,D,None,None,flags,(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,100,1e-7))
    print('OK',objshape,imgshape,useguess,'rms',r[0],'K',r[1].tolist(),'D',r[2].reshape(-1).tolist())
   except Exception as e: print('FAIL',objshape,imgshape,useguess,str(e).splitlines()[-1])
