# Four-tank benchmark

The four tank benchmark also known as the Quadruple Tank Porcess (QTP) is a toy system devoted to display regulators capabilities in control engineering. Here the QTP is used to show EasyMPC package to design a linear MPC controller.


## Process


## Program

```julia
### QTP Example ###

using EasyMPC
using DifferentialEquations
using Plots
using LinearAlgebra

### QTP with DifferentialEquation.jl
a1      = 1.31e-4; #m²
a2      = 1.51e-4; #m²
a3      = 9.27e-5; #m²
a4      = 8.82e-5; #m²
S       = 0.06; #m
g       = 9.81; #m²/s
gamma_a = 0.3;
gamma_b = 0.4;

qa = 0; #m3/h
qb = 0; #m3/h

function QTP(dh, h, p, t)
    dh[1] = -a1 / S * sqrt(2 * g * h[1]) + a3 / S * sqrt(2 * g * h[3]) + gamma_a / S * qa / 3600;
    dh[2] = -a2 / S * sqrt(2 * g * h[2]) + a4 / S * sqrt(2 * g * h[4]) + gamma_b / S * qb / 3600;
    dh[3] = -a3 / S * sqrt(2 * g * h[3]) + (1 - gamma_b) * qb / (3600 * S);
    dh[4] = -a4 / S * sqrt(2 * g * h[4]) + (1 - gamma_a) * qa / (3600 * S);
 end

tspan = (0.0,60.0);
tsteps = range(tspan[1], tspan[2], step = 0.1);
u0 = [0.68; 0.68; 0.68; 0.68];
prob = ODEProblem(QTP,u0,tspan);
sol = solve(prob, BS3(), saveat = tsteps);

plot(sol)

### Design the MPC controller to regulate the water level

xl = [0.65;0.66;0.65;0.66]
ul = [1.63 ; 2.00]

xcons = [0.2 1.36;
         0.2 1.36;
         0.2 1.30;
         0.2 1.30] #m
         
ucons = [0  3.26;
         0  4] #m3/h

Ts = 5;#s
method = "l"; #linear MPC
Q =  Matrix(I, 4, 4)
R = 0.01 *  Matrix(I, 2, 2)
N = 15; #horizon length
opt = "OSQP";

 f((x1, x2, x3, x4, u1, u2)) = 
 [ -a1 / S * sqrt(2 * g * x1) + a3 / S * sqrt(2 * g * x3) + gamma_a / S * u1 / 3600,
   -a2 / S * sqrt(2 * g * x2) + a4 / S * sqrt(2 * g * x4) + gamma_b / S * u2 / 3600,
   -a3 / S * sqrt(2 * g * x3) + (1 - gamma_b) * u2 / (3600 * S),
   -a4 / S * sqrt(2 * g * x4) + (1 - gamma_a) * u1 / (3600 * S),
 ]

RegulatorMPC = EasyMPC.MpcProblem(f, xl, ul, xcons, ucons, Ts, method, Q, R, N, opt)

state_reference = [0.3;0.3;0.301;0.306]
input_reference = [1.112;1.351]

obj = []
function ctrl_cb(int)
    RegulatorMPC.xt = copy(int.u)'
    RegulatorMPC.xref = state_reference .* ones(4, N+1)
    RegulatorMPC.uref = input_reference .* ones(2,N)
    u, x, xtilde, objc = SolveMpc(RegulatorMPC)
    global qa = u[1,1]
    global qb = u[2,1]
    println("qa : $qa")
    println("qb : $qb")
    push!(obj, objc)
end 

pcb  = PeriodicCallback(ctrl_cb, 5.0)


tspan = (0.0,1500.0);
tsteps2 = range(tspan[1], tspan[2], step = 0.01);

prob2 = ODEProblem(QTP,u0, tspan, callback = pcb);

### Run the QTP problem with the regulator
sol2 = solve(prob2, BS3());

plot(sol2)
plot(obj)
```

## Description

### Load packages

```julia
using EasyMPC
using DifferentialEquations
using Plots
using LinearAlgebra
```

[DifferentialEquations](https://diffeq.sciml.ai/stable/) is used to compute simulation of the QTP.

### QTP simulation

```julia
### QTP with DifferentialEquation.jl
a1      = 1.31e-4; #m²
a2      = 1.51e-4; #m²
a3      = 9.27e-5; #m²
a4      = 8.82e-5; #m²
S       = 0.06; #m
g       = 9.81; #m²/s
gamma_a = 0.3;
gamma_b = 0.4;

qa = 0; #m3/h
qb = 0; #m3/h

function QTP(dh, h, p, t)
    dh[1] = -a1 / S * sqrt(2 * g * h[1]) + a3 / S * sqrt(2 * g * h[3]) + gamma_a / S * qa / 3600;
    dh[2] = -a2 / S * sqrt(2 * g * h[2]) + a4 / S * sqrt(2 * g * h[4]) + gamma_b / S * qb / 3600;
    dh[3] = -a3 / S * sqrt(2 * g * h[3]) + (1 - gamma_b) * qb / (3600 * S);
    dh[4] = -a4 / S * sqrt(2 * g * h[4]) + (1 - gamma_a) * qa / (3600 * S);
 end

tspan = (0.0,60.0);
tsteps = range(tspan[1], tspan[2], step = 0.1);
u0 = [0.68; 0.68; 0.68; 0.68];
prob = ODEProblem(QTP,u0,tspan);
sol = solve(prob, BS3(), saveat = tsteps);

plot(sol)
```

QTP parameters are displayed, moreover the system is solved. The plot result shows that in order to regulate tank water level pumps flow need to be greater than 0.

### Design a linear MPC regulator

```julia
### Design the MPC controller to regulate the water level

xl = [0.65;0.66;0.65;0.66]
ul = [1.63 ; 2.00]

xcons = [0.2 1.36;
         0.2 1.36;
         0.2 1.30;
         0.2 1.30] #m
         
ucons = [0  3.26;
         0  4] #m3/h

Ts = 5;#s
method = "l"; #linear MPC
Q =  Matrix(I, 4, 4)
R = 0.01 *  Matrix(I, 2, 2)
N = 15; #horizon length
opt = "OSQP";

 f((x1, x2, x3, x4, u1, u2)) = 
 [ -a1 / S * sqrt(2 * g * x1) + a3 / S * sqrt(2 * g * x3) + gamma_a / S * u1 / 3600,
   -a2 / S * sqrt(2 * g * x2) + a4 / S * sqrt(2 * g * x4) + gamma_b / S * u2 / 3600,
   -a3 / S * sqrt(2 * g * x3) + (1 - gamma_b) * u2 / (3600 * S),
   -a4 / S * sqrt(2 * g * x4) + (1 - gamma_a) * u1 / (3600 * S),
 ]

RegulatorMPC = EasyMPC.MpcProblem(f, xl, ul, xcons, ucons, Ts, method, Q, R, N, opt)
```

### Regulator + differential equation solve

```julia
state_reference = [0.3;0.3;0.301;0.306]
input_reference = [1.112;1.351]

obj = []
function ctrl_cb(int)
    RegulatorMPC.xt = copy(int.u)'
    RegulatorMPC.xref = state_reference .* ones(4, N+1)
    RegulatorMPC.uref = input_reference .* ones(2,N)
    u, x, xtilde, objc = SolveMpc(RegulatorMPC)
    global qa = u[1,1]
    global qb = u[2,1]
    println("qa : $qa")
    println("qb : $qb")
    push!(obj, objc)
end 

pcb  = PeriodicCallback(ctrl_cb, 5.0)


tspan = (0.0,1500.0);
tsteps2 = range(tspan[1], tspan[2], step = 0.01);

prob2 = ODEProblem(QTP,u0, tspan, callback = pcb);
```

Here the regulator is called during the simulation with DifferentialEquation callback and `PeriodicCallback`. At each 5 seconds, the `ctrl_cb` is called, states measure, state and input references are updated within the Regulator and the MPC is solved. then fir inputs computed are sent to pumps qa and qb.

### Solving

Finallyn the linear MPC regulator and the simulation are simulated with the folowing:

```julia
### Run the QTP problem with the regulator
sol2 = solve(prob2, BS3());

plot(sol2)
plot(obj)
```

### Expected output

When running the program expected output are:



## Reference 

[1] Alvarado, I., Limon, D., De La Peña, D. M., Maestre, J. M., Ridao, M. A., Scheu, H., ... & Espinosa, J. (2011). A comparative analysis of distributed MPC techniques applied to the HD-MPC four-tank benchmark. Journal of Process Control, 21(5), 800-815.
  

