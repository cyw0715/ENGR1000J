import json, threading, time, urllib.request
HOST='192.168.137.244'
def get(p): return json.load(urllib.request.urlopen('http://'+HOST+p,timeout=3))
def post(x):
 r=urllib.request.Request('http://'+HOST+'/cmd',data=json.dumps(x).encode(),headers={'Content-Type':'application/json'},method='POST')
 return json.load(urllib.request.urlopen(r,timeout=.6))
class HB(threading.Thread):
 def __init__(self, start_seq=2): super().__init__(daemon=True); self.stop_evt=threading.Event(); self.seq=start_seq; self.err=None
 def run(self):
  while not self.stop_evt.is_set():
   try: post({'imu_heartbeat':True,'seq':self.seq}); self.seq+=1
   except Exception as e: self.err=e
   self.stop_evt.wait(.15)
 def stop(self): self.stop_evt.set(); self.join(1)
pre=get('/health')['mega_imu']; print('PRE',pre)
assert pre['mode']=='DISARMED' and pre['fault_reason']=='none'
print('ARM',post({'imu_arm':True}));
try:
 print('TARGET',post({'imu_set_delta':True,'seq':1,'fl':-5,'fr':5,'rl':-5,'rr':5}))
 hb=HB(start_seq=2); hb.start()
 start=time.monotonic(); rows=[]
 while time.monotonic()-start<4:
  q=get('/health')['mega_imu']; rows.append(q); print(round(time.monotonic()-start,2),q)
  if q['mode']=='FAULT': raise RuntimeError(q)
  if q['mode']=='ARMED' and all(abs(q['target_'+x]-q['applied_'+x])<=1 for x in ('fl','fr','rl','rr')): break
  time.sleep(.05)
 print('SAMPLES',len(rows),'ACTIVE_COUNT',sum(r['active'] for r in rows),'FINAL',rows[-1])
finally:
 hb.stop(); print('STOP',post({'imu_stop':True})); time.sleep(.3); print('POST',get('/health')['mega_imu'])
