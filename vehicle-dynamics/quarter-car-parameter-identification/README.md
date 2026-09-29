# Quarter-car parameter identification

A quarter-car dynamic model and parameter estimation using least squares (LS) and recursive least squares (RLS).

`RLS.m` contains a self-contained MATLAB simulation with time-varying mass and stiffness. `project1.m` defines model parameters for `project1sim.slx`. After running the Simulink model with an `out` result containing the expected signals, `LS.m` estimates constant parameters. The Simulink run and script compatibility have not been verified here.
