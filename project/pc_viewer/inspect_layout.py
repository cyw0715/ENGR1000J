from lander_console import LanderConsole

for geometry in ("1100x700", "1366x768", "1600x1000"):
    app = LanderConsole("manual-placeholder")
    app.withdraw()
    app.state("normal")
    app.geometry(geometry)
    app.update_idletasks()
    print("WINDOW", geometry, app.winfo_width(), app.winfo_height())
    for name in ("connect_button", "scan_button", "arm_button", "stop_button", "estop_button", "restore_button"):
        widget = getattr(app, name)
        parent = widget.master
        print(name, {
            "x": widget.winfo_x(), "y": widget.winfo_y(),
            "w": widget.winfo_width(), "h": widget.winfo_height(),
            "right": widget.winfo_x() + widget.winfo_width(),
            "bottom": widget.winfo_y() + widget.winfo_height(),
            "parent_w": parent.winfo_width(), "parent_h": parent.winfo_height(),
            "overflow_x": widget.winfo_x() + widget.winfo_width() > parent.winfo_width(),
            "overflow_y": widget.winfo_y() + widget.winfo_height() > parent.winfo_height(),
        })
    app.imu_command_worker.stop_event.set()
    app.destroy()
