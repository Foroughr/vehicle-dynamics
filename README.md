[README.md](https://github.com/user-attachments/files/32814982/README.md)
# Vehicle Dynamics and Suspension

Three MATLAB/Simulink course projects by Forough Sadat Razavi demonstrating mechanical-system modeling, vehicle handling, ride dynamics, suspension analysis, and parameter identification.

| Project | Focus |
| --- | --- |
| `lateral-vehicle-dynamics` | Two-degree-of-freedom bicycle model and steering/parameter studies |
| `quarter-car-ride-and-active-suspension` | Quarter-car model, road inputs, ride comfort, and passive versus PID active suspension |
| `quarter-car-parameter-identification` | Least-squares and recursive least-squares estimation of suspension parameters |

How to explore the projects

Download the repository and open each project folder in MATLAB. The .slx models require Simulink. The models were saved with MATLAB R2022b.

Adaptive path tracking: Run project2.m, simulate simproject2.slx, and then run project2_1.m with the simulation output available as out. The initializer selects 100 km/h by default. To examine the 60 and 80 km/h cases described in the coursework report, set Vx to 60/3.6 or 80/3.6 in the MATLAB workspace after running the initializer, then run the model and analysis again. Tracking error increases with speed in the reported simulations.

Adaptive cruise control: Run project3_data.m before project3.slx, then use project3_1.m to analyze the output out. The parameter script states a 30-second simulation, while the saved model has a 60-second stop time. To compare final values with the report's 30-second results, explicitly simulate for 30 seconds and use that output for the analysis.

Sliding-mode speed control: Open sim4.slx, add codes to the MATLAB path, and inspect codes/vehicle_setup.m before running the diagnostic scripts. The setup script saves changes to the model file. Diagnostic scripts generate result files and may remove previously generated figures or logs, so run them in a local working copy.

The reported results describe behavior under the assumptions of these simulation models. The numerical results have not been independently reproduced for this repository and do not constitute road-test validation.
