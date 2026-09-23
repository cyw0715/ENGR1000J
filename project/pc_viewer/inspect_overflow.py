from lander_console import LanderConsole

app = LanderConsole("manual-placeholder")
app.withdraw(); app.state("normal"); app.geometry("1100x700"); app.update_idletasks()
wx, wy = app.winfo_rootx(), app.winfo_rooty()
print("WINDOW", app.winfo_width(), app.winfo_height(), "requested", app.winfo_reqwidth(), app.winfo_reqheight())
rows=[]
def walk(w, depth=0):
    try:
        x=w.winfo_rootx()-wx; y=w.winfo_rooty()-wy; ww=w.winfo_width(); hh=w.winfo_height()
        reqw=w.winfo_reqwidth(); reqh=w.winfo_reqheight()
        if x+ww>app.winfo_width() or y+hh>app.winfo_height() or reqw>ww+20:
            rows.append((depth,w.winfo_class(),str(w),x,y,ww,hh,reqw,reqh,x+ww-app.winfo_width(),y+hh-app.winfo_height()))
    except Exception: pass
    for c in w.winfo_children(): walk(c,depth+1)
walk(app)
for row in rows: print(row)
app.imu_command_worker.stop_event.set(); app.destroy()
