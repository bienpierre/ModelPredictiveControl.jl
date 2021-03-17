# Linear Model Predictive Control

ModelPredictiveControl.jl provides linear Model Predictive Control formulation.
 
```math 
\underset{\textbf{u}} {\text{min}}~
\tilde{x}_N P \tilde{x}_N + \displaystyle\sum_{k=0}^{N-1} \tilde{x}_k Q \tilde{x}_k + \tilde{u}_k R \tilde{u}_k \\
\textnormal{s.t.} \ \  \tilde{x}_{k+1} =  A\tilde{x}_k + B\tilde{u}_k \\
\tilde{x}_k = x_k - x^r \\
\tilde{u}_k = u_k - u^r \\
x_k \in \mathcal{X} \\
u_k \in \mathcal{U} \\
x_0 = \bar{x}(t) 
```

## Basics

```jldoctest basics
julia> controller = mpc_problem(f, xl, ul, x_cons, u_cons, Ts, method, Q, R, N)

```
Where f is the non-linear dynamical system, xl and ul are the state and input linearised point, xcons and ucons are the state and input constraints, Ts is the sample time, method is the controller method, for a linear MPC "l", Q and R are the weighting matrices with appropriate dimensions, N is the horizon length.


It is also possible to design a regulator with a discrete state-space linear system:

```jldoctest basics
julia> controller = linear_mpc(Ad, Bd, xcons, ucons, Q, R, P, N)
```
Where Ad and Bd are the state and input matrices of the linear system, xcons and ucons are the state and input constraints, Q, R and P are the weighting matrices with appropriate dimensions, N is the horizon length.


Here the regulator is a composite with 

```julia
mutable struct Regulator
    formulation
    x_t
    x_ref
    u_ref
    method
end
```

Where controller is the JuMP model design, xt the states measurement, xref and uref are the states and inputs references, method is the controller MPC method, "l" for a linear MPC.

Then the MPC can be solved, however mesurement and references have to be filled in.

```jldoctest basics
julia> controller.x_t = state measurement
julia> controller.x_ref = state reference
julia> controller.u_ref = input reference
julia> x, u = solve_mpc(controller)
```

