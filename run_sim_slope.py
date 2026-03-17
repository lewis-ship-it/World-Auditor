# FILE: run_slope_sim.py
from alignment_core.sensors.imu import IMUSensor
# (Assumes other imports are same as run_dynamic_sim)

def run_sim():
    # ... (Setup robot and kernels same as before) ...
    imu = IMUSensor(robot)
    
    # Imagine the robot is on a 15 degree hill
    real_hill_angle = 15.0
    
    # THE AI ESTIMATOR AT WORK:
    detected_slope = imu.estimate_slope(real_hill_angle)
    print(f"IMU Sensor Detected Slope: {detected_slope}°")
    
    # The AI now uses the DETECTED slope for its audit
    audit = auditor.audit_intent(v_target=10, r_target=5, a_target=0, slope=detected_slope)
    
    print(f"Decision with Estimated Slope: {audit['summary']}")

if __name__ == "__main__":
    # Ensure you initialize your kernels here before calling
    pass