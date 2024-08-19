**Part A: Pressure Control**
Design a PD controller to regulate the fuel tank pressure of a rocket. The goal is to adjust the pressure flow rate to keep the tank pressure within safe limits.

**Function:** pressure_pd_solution(delta_t, current_pressure, target_pressure, data)
**Part B:** Thrust Control
Implement a PID controller to manage the rocket's throttle, ensuring it follows a specified velocity profile and lands safely.

**Function: **rocket_pid_solution(delta_t, current_velocity, optimal_velocity, data)
**Setup and Usage**
Clone the repository and install dependencies.
Implement the controllers in RocketPIDStudent_submission.py.
Test your solutions using RocketPID_tester.py.
Visualize results with --showgraph (matplotlib or turtle).
