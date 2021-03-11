"""
Solve MPC controller 

    #Parameters
  
    -A MPC regulator method with x_t, x_ref and u_ref values.
    
    #Example
    
    u, x = solve_mpc(controller)

"""


function solve_mpc(controller)
    if controller.method == "l" 
        # The variable are retrieved from the model 
        x = controller.formulation[:x]
        x_tilde = controller.formulation[:x_tilde]
        x_ref = controller.formulation[:x_ref]
        u = controller.formulation[:u]
        u_tilde = controller.formulation[:u_tilde]
        u_ref = controller.formulation[:u_ref]
        x_ref_update = controller.x_ref
        u_ref_update = controller.u_ref 
        
        # The JuMP model is updated 
        JuMP.fix.(x[:,1], controller.x_t[:,1]; force = true)
        for k in 1 : 1 : size(u,2)
            JuMP.fix.(u_ref[:,k], u_ref_update[:,k]; force = true)
        end
        for k in 1 : 1 : size(x,2)
            JuMP.fix.(x_ref[:,k], x_ref_update[:,k]; force = true)
        end

        # The model is optimized
        JuMP.optimize!(controller.formulation)
        
        # The optimised values are retrieved
        u_optimised = JuMP.value.(u[:,:]) 
        u_tilde_optimised = JuMP.value.(u_tilde[:,:]) 
        x_optimised = JuMP.value.(x[:,:])  
        x_tilde_optimised = JuMP.value.(x_tilde[:,:]) 
        obj_optimised = JuMP.objective_value(controller.formulation)
    end
    return u_optimised, x_optimised, u_tilde_optimised, x_tilde_optimised, obj_optimised
end

