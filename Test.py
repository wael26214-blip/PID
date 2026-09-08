import time
from PIDController import *

if __name__ == "__main__":
    pid = PIDController(
                        dt=0.1,
                        kp=3,
                        ki=0.1,
                        kd=0.2,
                        min_output=-10.0,
                        max_output=10.0,
                        )
    target_setpoint = 15.0
    current_state = 0.0
    dt = pid.dt
    
    pid.set_active_target(target_setpoint)
    deadzone=pid.deadzone
    
    print(f"Starting simulation. Target: {target_setpoint}, Initial State: {current_state}\n")

    loop_number = 1
    
    while (True):

        error = target_setpoint - current_state
        
        pid_output = pid.compute(current_state)
        
        current_state += pid_output * dt
        
        print(f"***** Loop {loop_number :2d} *****")
        print(f"  Error:  {error:7.4f}")
        print(f"  Output: {pid_output:7.4f}")
        print(f"  State:  {current_state:7.4f}")
        print("-" * 30)
        time.sleep(0.02)

        loop_number +=1

