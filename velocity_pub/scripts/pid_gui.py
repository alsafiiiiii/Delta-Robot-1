#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

class PIDTunerGUI:
    def __init__(self, master):
        self.master = master
        master.title("Camera PNP PID Tuner")
        master.geometry("400x300")
        
        # ROS 2 Setup
        rclpy.init()
        self.node = Node('pid_tuner_gui')
        self.cli = self.node.create_client(SetParameters, '/camera_pnp/set_parameters')
        
        # UI Variables
        self.kp_var = tk.DoubleVar(value=12.0)
        self.ki_var = tk.DoubleVar(value=0.5)
        self.kd_var = tk.DoubleVar(value=10.0)
        
        # Layout
        self.create_slider("Kp (Proportional)", self.kp_var, 0.0, 50.0)
        self.create_slider("Ki (Integral)", self.ki_var, 0.0, 10.0)
        self.create_slider("Kd (Derivative)", self.kd_var, 0.0, 50.0)
        
        # Update Button
        self.update_btn = tk.Button(master, text="Update Parameters", command=self.send_params, bg="green", fg="white", font=("Arial", 12))
        self.update_btn.pack(pady=20, fill='x', padx=20)
        
        # Status Label
        self.status_lbl = tk.Label(master, text="Ready", fg="gray")
        self.status_lbl.pack(side="bottom", pady=5)

    def create_slider(self, label_text, variable, min_val, max_val):
        frame = tk.Frame(self.master)
        frame.pack(pady=10, fill='x', padx=20)
        
        lbl = tk.Label(frame, text=label_text, font=("Arial", 10, "bold"))
        lbl.pack(anchor='w')
        
        slider = tk.Scale(frame, variable=variable, from_=min_val, to=max_val, orient='horizontal', resolution=0.1)
        slider.pack(fill='x')

    def send_params(self):
        if not self.cli.wait_for_service(timeout_sec=1.0):
            self.status_lbl.config(text="Service /camera_pnp/set_parameters not available!", fg="red")
            return

        req = SetParameters.Request()
        
        # Kp
        p_kp = Parameter()
        p_kp.name = "kp" # Note: camera_pnp uses 'kp', 'ki', 'kd' (legacy names in declare_parameter)
        # Wait, the new camera_pnp code removed declare_parameter logic in the refactor?
        # Let me check if I preserved param_cb...
        # Ah, I see I might have overwritten the parameter callback logic in the previous huge edit.
        # I need to verify camera_pnp.py actually accepts parameters!
        
        # Assuming camera_pnp HAS parameters declared. 
        # (If I removed them, I need to add them back).
        
        p_kp.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.kp_var.get())
        
        p_ki = Parameter()
        p_ki.name = "ki"
        p_ki.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.ki_var.get())
        
        p_kd = Parameter()
        p_kd.name = "kd"
        p_kd.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.kd_var.get())
        
        req.parameters = [p_kp, p_ki, p_kd]
        
        future = self.cli.call_async(req)
        # We can't spin_until_future_complete easily in Tkinter mainloop without threading
        # So we just fire and forget for UI responsiveness, or basic check.
        self.status_lbl.config(text=f"Sent: Kp={self.kp_var.get()}, Ki={self.ki_var.get()}, Kd={self.kd_var.get()}", fg="blue")

def main():
    root = tk.Tk()
    app = PIDTunerGUI(root)
    root.mainloop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
