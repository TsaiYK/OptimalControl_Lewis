# OptimalControl_Lewis

Chapter 1: Static optimization
Chapter 2: Discrete-time system control

This repository is to give you the example of the discrete-time system control from Lewis et al. (2012)

1. 'main_ch1.m' and 'main_ch2.m' are the scripts for demonstating the math equations from the textbook.
2. 'main_ch2_N_equal_1.m' and 'main_ch2_N_equal_2.m' are the scripts for getting the analytical form of control strategies by setting the time horizons as 1 and 2, respectively. These are a little different from the mathematical formulation from the book. The reference of the mathematical derivation is from Rawlings et al. (2017) which is based on finite-horizon programming.
3. 'main_ch2_DigitalControl_RC_circuit.m' is the demo of digital control of the RC circuit system, which is a continuous-time system controlled by discretized control commands.  Given the sample of time period T, the discret form of the system can be obtained.
4. 'main_DP.m' is for dynamic programming (DP) of the feedback control strategy by a graident-based optimizer, sqp.
5. 'cost_func.m' defines the cost function used for the DP.
