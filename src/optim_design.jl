"""
Linear MPC Controller 
    
    # Parameters
    
    -A discrete state-space system with states and inputs constraints (states are considered available)
    -Weithing matrices with appropriate dimension Q and R and P
    _Horizon length N
    _Quadratic programming solver
    
    # Examples
   
    method = linear_mpc(sys, Q, R, P, N, opt)
    
"""

mutable struct Regulator
    formulation
    x_t::AbstractVector
    x_ref::AbstractMatrix
    u_ref::AbstractMatrix
    method::String
end

function linear_mpc(A::AbstractMatrix, B::AbstractMatrix, x_cons::AbstractMatrix, u_cons::AbstractMatrix, Q::AbstractMatrix, R::AbstractMatrix, P::AbstractMatrix, N::Integer)
    C = JuMP.Model(OSQP.Optimizer)
    JuMP.set_optimizer_attribute(C, "verbose", false)
    
    JuMP.@variables(C, begin
        x[1:size(A,1), 1:N + 1]
        x_tilde[1:size(A,1), 1:N + 1]
        x_ref[1:size(A,1), 1:N + 1]
        u[1:size(B,2), 1:N]
        u_tilde[1:size(B,2), 1:N]
        u_ref[1:size(B,2), 1:N]
    end)
    
    for k in 1 : 1 : N
        JuMP.@constraint(C, x_tilde[:,k+1] .== A * x_tilde[:,k] + B * u_tilde[:,k])
    end
    
    for k in 1 : 1 : N
        JuMP.@constraint(C, u_cons[:,1] .<= u[:,k] .<= u_cons[:,2])
    end
    
    for k in 1 : 1 : N  
        JuMP.@constraint(C, x_cons[:,1] .<= x[:,k] .<= x_cons[:,2])
    end
    
    #JuMP.@constraint(C, -0.1 .<= xtilde[:,end] .<= 0.1) #terminal constraint to mimic Xf = 0, should be improved with maximum invariant set computation
    
    for k in 1 : 1 : N + 1    
        JuMP.@constraint(C, x_tilde[:,k] .== x[:,k] .- x_ref[:,k])
    end
    
    for k in 1 : 1 : N
        JuMP.@constraint(C, u_tilde[:,k] .== u[:, k] .- u_ref[:,k])
    end
    
    JuMP.@objective(C, Min, x_tilde[:,end]' * P * x_tilde[:,end] + sum(x_tilde[:,k]' * Q * x_tilde[:,k] + u_tilde[:,k]' * R * u_tilde[:,k] for k in 1 : N))
         
    return Regulator(C, zeros(size(A,1)), zeros(size(A,1),N), zeros(size(B,2),N), "l")
end



