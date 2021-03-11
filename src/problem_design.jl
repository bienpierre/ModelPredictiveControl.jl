"""
MPC controller 

    #Parameters
    
    -A differential equation x' = f(x,u)
    -States and inputs linearization points
    -States and inputs constraints
    -A sample time for discretization
    -A method, 'l'
    -Weithing matrices with appropriate dimension Q and R and P
    _Horizon length N
    
    #Example
    
    regulator = mpc_problem(f, x_cons, u_cons, Ts, method, Q, R, P, N)

"""

function mpc_problem(f, xl::AbstractArray, ul::AbstractArray, x_cons::AbstractArray, u_cons::AbstractArray, Ts::Float64, method::String, Q::AbstractArray, R::Array, N::Integer)
    if method == "l"
        # the dynamical system is linearised
        JacobM = ForwardDiff.jacobian(f, vcat(xl, ul))
        A = JacobM[begin:size(xl,1), begin:size(xl,1)]
        B = JacobM[begin:size(xl,1), size(xl,1)+1:end]
        
        # the linear system is dicretized
        sysc = ControlSystems.ss(A, B,  Matrix(LinearAlgebra.I, size(A,1), size(A,1)), zeros(size(B,1),size(B,2)))
        sysd = ControlSystems.c2d(sysc, Ts)
        
        # The tmerinal weighting matrix is computed
        P = ModelPredictiveControl.terminal_weight(sysd.A, sysd.B, Q, R)
        controller = ModelPredictiveControl.linear_mpc(sysd.A, sysd.B, x_cons, u_cons, Q, R, P, N)
    end
        
    return controller 

end


"""
Terminal matrix computation of the terminal cost x'Px. P is the solution to the discrete-time algebraic Riccati equation.

    #Parameters
    -State and input discrete matrices A and B
    _Weighting matrices Q and R
    
    #Example
    
    P = terminal_weight(Ad, Bd, Q, R)

"""

function terminal_weight(Ad::AbstractMatrix, Bd::AbstractMatrix, Q::AbstractMatrix, R::AbstractMatrix)
    return ControlSystems.dare(Ad, Bd, Q, R)
end


