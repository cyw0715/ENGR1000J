from lander_console import LanderConsole

for geometry in ("1100x700+4000+4000", "1366x768+4000+4000", "1600x1000+4000+4000"):
    app=LanderConsole("manual-placeholder")
    app.state("normal"); app.attributes('-alpha',0.0); app.geometry(geometry); app.update()
    print('WINDOW',geometry,app.winfo_width(),app.winfo_height(),'REQ',app.winfo_reqwidth(),app.winfo_reqheight())
    for name in ('connect_button','scan_button','arm_button','stop_button','estop_button','restore_button'):
        w=getattr(app,name); p=w.master
        print(name,{'root':(w.winfo_rootx()-app.winfo_rootx(),w.winfo_rooty()-app.winfo_rooty()),'size':(w.winfo_width(),w.winfo_height()),'parent':(p.winfo_width(),p.winfo_height()),'xy':(w.winfo_x(),w.winfo_y()),'right':w.winfo_x()+w.winfo_width()})
    app.imu_command_worker.stop_event.set(); app.destroy()
