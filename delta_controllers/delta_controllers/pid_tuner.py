#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import numpy as np

class BatchTuner(Node):
    def __init__(self):
        super().__init__('batch_tuner')
        
        # Subscribe to robot performance
        self.sub = self.create_subscription(Float32MultiArray, '/pnp/debug', self.data_cb, 10)
        
        # Client to update robot gains
        self.cli = self.create_client(SetParameters, '/camera_pnp/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for camera_pnp service...')
        
        # --- TUNING SETTINGS ---
        self.BATCH_SIZE = 5  # Number of boxes to verify before changing ANYTHING
        
        # Current Gains (Start with your "Golden" values)
        self.sprint_kp = 15.0   
        self.brake_kp = 6.0
        self.brake_kd = 0.35
        
        # Batch Data Buffers
        self.batch_wobbles = []
        self.batch_accuracies = []
        
        # State
        self.history = []
        self.recording = False
        self.cooldown = False
        
        print("=== BATCH VERIFICATION TUNER STARTED ===")
        print(f"Strategy: Test gains on {self.BATCH_SIZE} boxes before deciding.")
        print(f"Starting Gains: Sprint={self.sprint_kp}, BrakeP={self.brake_kp}, BrakeD={self.brake_kd}")
        self.push_all_gains() # Ensure robot matches tuner start

    def data_cb(self, msg):
        err_mm = abs(msg.data[0])
        
        if self.cooldown:
            if err_mm < 5 or msg.data[2] == 0.0: 
                self.cooldown = False
                print("Ready for next box...")
            return

        if not self.recording and err_mm > 10:
            self.recording = True
            self.history = []
            print(f"Box {len(self.batch_wobbles)+1}/{self.BATCH_SIZE} detected! Recording...")
            
        if self.recording:
            self.history.append(err_mm)
            if (len(self.history) > 10 and err_mm < 2.0) or len(self.history) > 100:
                self.process_run()
                self.recording = False
                self.cooldown = True

    def process_run(self):
        data = np.array(self.history)
        
        # Calculate Single Run Metrics
        wobble = np.sum(np.abs(np.diff(data))) - (data[0] - data[-1])
        acc = np.mean(data[-5:])
        
        print(f"   -> Run Result: Wobble={wobble:.1f}, Acc={acc:.1f}mm")
        
        # Add to Batch
        self.batch_wobbles.append(wobble)
        self.batch_accuracies.append(acc)
        
        # CHECK IF BATCH COMPLETE
        if len(self.batch_wobbles) >= self.BATCH_SIZE:
            self.analyze_batch()

    def analyze_batch(self):
        # Calculate Averages
        avg_wobble = np.mean(self.batch_wobbles)
        avg_acc = np.mean(self.batch_accuracies)
        
        print("\n====== BATCH COMPLETE ======")
        print(f"AVG Wobble: {avg_wobble:.1f} (Target < 50)")
        print(f"AVG Accuracy: {avg_acc:.1f}mm (Target < 3mm)")
        
        # === TUNING LOGIC (Based on Averages) ===
        
        # 1. FAIL: Too Unstable
        if avg_wobble > 50.0:
            print(">> FAIL: System is too shaky on average.")
            if self.brake_kd < 0.85:
                self.brake_kd += 0.05
                print(f"   ACTION: Increasing Damping (Kd) to {self.brake_kd:.2f}")
            else:
                self.brake_kp -= 2.0 # Drop P faster since we failed a whole batch
                print(f"   ACTION: Damping Maxed. Dropping Stiffness (Kp) to {self.brake_kp:.1f}")

        # 2. FAIL: Too Inaccurate
        elif avg_acc > 3.0:
            print(">> FAIL: System is consistently inaccurate.")
            self.brake_kp += 1.0
            print(f"   ACTION: Increasing Stiffness (Kp) to {self.brake_kp:.1f}")
            
        # 3. PASS: Solid Performance
        else:
            print(">> PASS: Batch was successful!")
            self.sprint_kp += 1.0 # Careful increase
            print(f"   ACTION: Boosting Speed (Sprint Kp) to {self.sprint_kp:.1f}")

        # Apply New Gains & Reset Batch
        self.push_all_gains()
        self.batch_wobbles = []
        self.batch_accuracies = []
        print("==============================\n")

    def push_all_gains(self):
        self.update_gain("sprint_kp", self.sprint_kp)
        self.update_gain("brake_kp", self.brake_kp)
        self.update_gain("brake_kd", self.brake_kd)

    def update_gain(self, name, value):
        req = SetParameters.Request()
        param = Parameter()
        param.name = name
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        req.parameters = [param]
        self.cli.call_async(req)

def main():
    rclpy.init()
    node = BatchTuner()
    rclpy.spin(node)

if __name__ == '__main__':
    main()