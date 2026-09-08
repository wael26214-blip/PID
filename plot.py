#الكود ده علشان الطباعة جبته زي م هو معرفش بيشتغل إزاي





import time

from PIDController import PIDController

import matplotlib.pyplot as plt



def run_and_plot():

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

   

    loop_numbers = []

    states = []

    outputs = []

   

    for loop_number in range(1, 1001):

        pid_output = pid.compute(current_state)

        current_state += pid_output * dt

       

        loop_numbers.append(loop_number)

        states.append(current_state)

        outputs.append(pid_output)



    # رسم الجراف البياني في نافذة مستقلة

    plt.figure(figsize=(12, 6))

   

    # رسم منحنى استجابة الروبوت مقارنة بالتارجت

    plt.subplot(2, 1, 1)

    plt.plot(loop_numbers, states, label='Robot State', color='b', linewidth=2)

    plt.axhline(y=target_setpoint, color='r', linestyle='--', label='Target')

    plt.title('PID Simulation - System Response')

    plt.ylabel('State Value')

    plt.legend()

    plt.grid(True)

   

    # رسم منحنى خروج الكنتروللر (PID Output)

    plt.subplot(2, 1, 2)

    plt.plot(loop_numbers, outputs, label='PID Output', color='g', linewidth=1.5)

    plt.xlabel('Loop Number')

    plt.ylabel('Control Output')

    plt.legend()

    plt.grid(True)

   

    plt.tight_layout()

    plt.show()



if __name__ == "__main__":

    run_and_plot()
    
