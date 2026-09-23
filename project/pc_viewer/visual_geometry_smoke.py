from lander_console import LanderConsole, CONTROL_MODE_IMU

app=LanderConsole('manual-placeholder')
app.attributes('-alpha',0.0); app.state('normal'); app.update()
app.control_mode_var.set(CONTROL_MODE_IMU); app._on_control_mode_changed(); app.update()
W,H=app.winfo_width(),app.winfo_height()
checks={}
for name in ('connect_button','scan_button','arm_button','stop_button','estop_button','restore_button'):
    w=getattr(app,name)
    x=w.winfo_rootx()-app.winfo_rootx(); y=w.winfo_rooty()-app.winfo_rooty()
    checks[name]={'bounds':(x,y,w.winfo_width(),w.winfo_height()),'inside':x>=0 and y>=0 and x+w.winfo_width()<=W and y+w.winfo_height()<=H}
print({'window':(W,H),'requested':(app.winfo_reqwidth(),app.winfo_reqheight()),'screen':(app.winfo_screenwidth(),app.winfo_screenheight()),'scaling':app.ui_scaling,'titles':[v.get() for v in app.panel_title_vars],'buttons':checks})
ok=all(v['inside'] for v in checks.values()) and app.winfo_reqwidth()<=W and app.winfo_reqheight()<=H
app.imu_command_worker.stop_event.set(); app.destroy(); raise SystemExit(0 if ok else 2)
