var documenterSearchIndex = {"docs":
[{"location":"MPC/#Model-Predictive-Control","page":"Model Predictive Control","title":"Model Predictive Control","text":"","category":"section"},{"location":"MPC/#Building-MPC-problems","page":"Model Predictive Control","title":"Building MPC problems","text":"","category":"section"},{"location":"MPC/","page":"Model Predictive Control","title":"Model Predictive Control","text":"EasyMPC provides linear Model Predictive Control formualtion.","category":"page"},{"location":"functions/#Functions-user-guide","page":"Functions user guide","title":"Functions user guide","text":"","category":"section"},{"location":"functions/","page":"Functions user guide","title":"Functions user guide","text":"This section describes functions references.","category":"page"},{"location":"functions/","page":"Functions user guide","title":"Functions user guide","text":"EasyMPC.MpcProblem\nEasyMPC.LinearMpc\nEasyMPC.SolveMpc\nEasyMPC.TerminalWeight","category":"page"},{"location":"functions/","page":"Functions user guide","title":"Functions user guide","text":"MpcProblem","category":"page"},{"location":"functions/","page":"Functions user guide","title":"Functions user guide","text":"MpcProblem()","category":"page"},{"location":"style_guide/#Style-guide","page":"Style guide","title":"Style guide","text":"","category":"section"},{"location":"style_guide/","page":"Style guide","title":"Style guide","text":"JuMP style guide.","category":"page"},{"location":"Examples/QTP/#Four-tank-benchmark","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"","category":"section"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"The four tank benchmark also known as the Quadruple Tank Porcess (QTP) is a toy system devoted to display regulators capabilities in control engineering. Here the QTP is used to show EasyMPC package to design a linear MPC controller.","category":"page"},{"location":"Examples/QTP/#Process","page":"Four-tank benchmark","title":"Process","text":"","category":"section"},{"location":"Examples/QTP/#Program","page":"Four-tank benchmark","title":"Program","text":"","category":"section"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"### QTP Example ###\n\nusing EasyMPC\nusing DifferentialEquations\nusing Plots\nusing LinearAlgebra\n\n### QTP with DifferentialEquation.jl\na1      = 1.31e-4; #m²\na2      = 1.51e-4; #m²\na3      = 9.27e-5; #m²\na4      = 8.82e-5; #m²\nS       = 0.06; #m\ng       = 9.81; #m²/s\ngamma_a = 0.3;\ngamma_b = 0.4;\n\nqa = 0; #m3/h\nqb = 0; #m3/h\n\nfunction QTP(dh, h, p, t)\n    dh[1] = -a1 / S * sqrt(2 * g * h[1]) + a3 / S * sqrt(2 * g * h[3]) + gamma_a / S * qa / 3600;\n    dh[2] = -a2 / S * sqrt(2 * g * h[2]) + a4 / S * sqrt(2 * g * h[4]) + gamma_b / S * qb / 3600;\n    dh[3] = -a3 / S * sqrt(2 * g * h[3]) + (1 - gamma_b) * qb / (3600 * S);\n    dh[4] = -a4 / S * sqrt(2 * g * h[4]) + (1 - gamma_a) * qa / (3600 * S);\n end\n\ntspan = (0.0,60.0);\ntsteps = range(tspan[1], tspan[2], step = 0.1);\nu0 = [0.68; 0.68; 0.68; 0.68];\nprob = ODEProblem(QTP,u0,tspan);\nsol = solve(prob, BS3(), saveat = tsteps);\n\nplot(sol)\n\n### Design the MPC controller to regulate the water level\n\nxl = [0.65;0.66;0.65;0.66]\nul = [1.63 ; 2.00]\n\nxcons = [0.2 1.36;\n         0.2 1.36;\n         0.2 1.30;\n         0.2 1.30] #m\n         \nucons = [0  3.26;\n         0  4] #m3/h\n\nTs = 5;#s\nmethod = \"l\"; #linear MPC\nQ =  Matrix(I, 4, 4)\nR = 0.01 *  Matrix(I, 2, 2)\nN = 15; #horizon length\nopt = \"OSQP\";\n\n f((x1, x2, x3, x4, u1, u2)) = \n [ -a1 / S * sqrt(2 * g * x1) + a3 / S * sqrt(2 * g * x3) + gamma_a / S * u1 / 3600,\n   -a2 / S * sqrt(2 * g * x2) + a4 / S * sqrt(2 * g * x4) + gamma_b / S * u2 / 3600,\n   -a3 / S * sqrt(2 * g * x3) + (1 - gamma_b) * u2 / (3600 * S),\n   -a4 / S * sqrt(2 * g * x4) + (1 - gamma_a) * u1 / (3600 * S),\n ]\n\nRegulatorMPC = EasyMPC.MpcProblem(f, xl, ul, xcons, ucons, Ts, method, Q, R, N, opt)\n\nstate_reference = [0.3;0.3;0.301;0.306]\ninput_reference = [1.112;1.351]\n\nobj = []\nfunction ctrl_cb(int)\n    RegulatorMPC.xt = copy(int.u)'\n    RegulatorMPC.xref = state_reference .* ones(4, N+1)\n    RegulatorMPC.uref = input_reference .* ones(2,N)\n    u, x, xtilde, objc = SolveMpc(RegulatorMPC)\n    global qa = u[1,1]\n    global qb = u[2,1]\n    println(\"qa : $qa\")\n    println(\"qb : $qb\")\n    push!(obj, objc)\nend \n\npcb  = PeriodicCallback(ctrl_cb, 5.0)\n\n\ntspan = (0.0,1500.0);\ntsteps2 = range(tspan[1], tspan[2], step = 0.01);\n\nprob2 = ODEProblem(QTP,u0, tspan, callback = pcb);\n\n### Run the QTP problem with the regulator\nsol2 = solve(prob2, BS3());\n\nplot(sol2)\nplot(obj)","category":"page"},{"location":"Examples/QTP/#Description","page":"Four-tank benchmark","title":"Description","text":"","category":"section"},{"location":"Examples/QTP/#Load-packages","page":"Four-tank benchmark","title":"Load packages","text":"","category":"section"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"using EasyMPC\nusing DifferentialEquations\nusing Plots\nusing LinearAlgebra","category":"page"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"DifferentialEquations is used to compute simulation of the QTP.","category":"page"},{"location":"Examples/QTP/#QTP-simulation","page":"Four-tank benchmark","title":"QTP simulation","text":"","category":"section"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"### QTP with DifferentialEquation.jl\na1      = 1.31e-4; #m²\na2      = 1.51e-4; #m²\na3      = 9.27e-5; #m²\na4      = 8.82e-5; #m²\nS       = 0.06; #m\ng       = 9.81; #m²/s\ngamma_a = 0.3;\ngamma_b = 0.4;\n\nqa = 0; #m3/h\nqb = 0; #m3/h\n\nfunction QTP(dh, h, p, t)\n    dh[1] = -a1 / S * sqrt(2 * g * h[1]) + a3 / S * sqrt(2 * g * h[3]) + gamma_a / S * qa / 3600;\n    dh[2] = -a2 / S * sqrt(2 * g * h[2]) + a4 / S * sqrt(2 * g * h[4]) + gamma_b / S * qb / 3600;\n    dh[3] = -a3 / S * sqrt(2 * g * h[3]) + (1 - gamma_b) * qb / (3600 * S);\n    dh[4] = -a4 / S * sqrt(2 * g * h[4]) + (1 - gamma_a) * qa / (3600 * S);\n end\n\ntspan = (0.0,60.0);\ntsteps = range(tspan[1], tspan[2], step = 0.1);\nu0 = [0.68; 0.68; 0.68; 0.68];\nprob = ODEProblem(QTP,u0,tspan);\nsol = solve(prob, BS3(), saveat = tsteps);\n\nplot(sol)","category":"page"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"QTP parameters are displayed, moreover the system is solved. The plot result shows that in order to regulate tank water level pumps flow need to be greater than 0.","category":"page"},{"location":"Examples/QTP/#Design-a-linear-MPC-regulator","page":"Four-tank benchmark","title":"Design a linear MPC regulator","text":"","category":"section"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"### Design the MPC controller to regulate the water level\n\nxl = [0.65;0.66;0.65;0.66]\nul = [1.63 ; 2.00]\n\nxcons = [0.2 1.36;\n         0.2 1.36;\n         0.2 1.30;\n         0.2 1.30] #m\n         \nucons = [0  3.26;\n         0  4] #m3/h\n\nTs = 5;#s\nmethod = \"l\"; #linear MPC\nQ =  Matrix(I, 4, 4)\nR = 0.01 *  Matrix(I, 2, 2)\nN = 15; #horizon length\nopt = \"OSQP\";\n\n f((x1, x2, x3, x4, u1, u2)) = \n [ -a1 / S * sqrt(2 * g * x1) + a3 / S * sqrt(2 * g * x3) + gamma_a / S * u1 / 3600,\n   -a2 / S * sqrt(2 * g * x2) + a4 / S * sqrt(2 * g * x4) + gamma_b / S * u2 / 3600,\n   -a3 / S * sqrt(2 * g * x3) + (1 - gamma_b) * u2 / (3600 * S),\n   -a4 / S * sqrt(2 * g * x4) + (1 - gamma_a) * u1 / (3600 * S),\n ]\n\nRegulatorMPC = EasyMPC.MpcProblem(f, xl, ul, xcons, ucons, Ts, method, Q, R, N, opt)","category":"page"},{"location":"Examples/QTP/#Regulator-differential-equation-solve","page":"Four-tank benchmark","title":"Regulator + differential equation solve","text":"","category":"section"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"state_reference = [0.3;0.3;0.301;0.306]\ninput_reference = [1.112;1.351]\n\nobj = []\nfunction ctrl_cb(int)\n    RegulatorMPC.xt = copy(int.u)'\n    RegulatorMPC.xref = state_reference .* ones(4, N+1)\n    RegulatorMPC.uref = input_reference .* ones(2,N)\n    u, x, xtilde, objc = SolveMpc(RegulatorMPC)\n    global qa = u[1,1]\n    global qb = u[2,1]\n    println(\"qa : $qa\")\n    println(\"qb : $qb\")\n    push!(obj, objc)\nend \n\npcb  = PeriodicCallback(ctrl_cb, 5.0)\n\n\ntspan = (0.0,1500.0);\ntsteps2 = range(tspan[1], tspan[2], step = 0.01);\n\nprob2 = ODEProblem(QTP,u0, tspan, callback = pcb);","category":"page"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"Here the regulator is called during the simulation with DifferentialEquation callback and PeriodicCallback. At each 5 seconds, the ctrl_cb is called, states measure, state and input references are updated within the Regulator and the MPC is solved. then fir inputs computed are sent to pumps qa and qb.","category":"page"},{"location":"Examples/QTP/#Solving","page":"Four-tank benchmark","title":"Solving","text":"","category":"section"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"Finallyn the linear MPC regulator and the simulation are simulated with the folowing:","category":"page"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"### Run the QTP problem with the regulator\nsol2 = solve(prob2, BS3());\n\nplot(sol2)\nplot(obj)","category":"page"},{"location":"Examples/QTP/#Expected-output","page":"Four-tank benchmark","title":"Expected output","text":"","category":"section"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"When running the program expected output are:","category":"page"},{"location":"Examples/QTP/#Reference","page":"Four-tank benchmark","title":"Reference","text":"","category":"section"},{"location":"Examples/QTP/","page":"Four-tank benchmark","title":"Four-tank benchmark","text":"[1] Alvarado, I., Limon, D., De La Peña, D. M., Maestre, J. M., Ridao, M. A., Scheu, H., ... & Espinosa, J. (2011). A comparative analysis of distributed MPC techniques applied to the HD-MPC four-tank benchmark. Journal of Process Control, 21(5), 800-815.","category":"page"},{"location":"linearmpc/#Linear-Model-Predictive-Control","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"","category":"section"},{"location":"linearmpc/","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"EasyMPC provides linear Model Predictive Control formulation.","category":"page"},{"location":"linearmpc/#Basics","page":"Linear Model Predictive Control","title":"Basics","text":"","category":"section"},{"location":"linearmpc/","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"julia> Regulator = EasyMPC.MpcProblem(f, xl, ul, xcons, ucons, Ts, method, Q, R, N, opt)\n","category":"page"},{"location":"linearmpc/","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"Where f is the non-lienar dynamical system, xl and ul are the state and input linearised point, xcons and ucons are the state and input constraints, Ts is the sample time, method is the controller method, for a linear MPC \"l\", Q and R are the weighting matrices with appropriate dimensions, N is the horizon length.","category":"page"},{"location":"linearmpc/","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"It is also possible to design a regulator with a discrete state-space linear system:","category":"page"},{"location":"linearmpc/","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"julia> Regulator = EasyMPC.LinearMpc(Ad, Bd, xcons, ucons, Q, R, P, N, opt)","category":"page"},{"location":"linearmpc/","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"Where ad and Bd are the state and input matrices of the linear system, xcons and ucons are the state and input constraints, Q, R and P are the weighting matrices with appropriate dimensions, N is the horizon length.","category":"page"},{"location":"linearmpc/","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"Here the regulator is a composite with ","category":"page"},{"location":"linearmpc/","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"mutable struct MpcController\n    controller\n    xt\n    xref\n    uref\n    method\nend","category":"page"},{"location":"linearmpc/","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"Where controller is the JuMP model design, xt the states measurement, xref and uref are the states and inputs references, method is the controller MPC method, \"l\" for a linear MPC.","category":"page"},{"location":"linearmpc/","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"Then a MPC can be solved, however mesurement and references have to be fill in.","category":"page"},{"location":"linearmpc/","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"julia> Regulator.xt = xt\njulia> Regulator.xref = xref\njulia> Regulator.uref = uref\njulia> x, u = EasyMPC.SolveMpc(Regulator)","category":"page"},{"location":"linearmpc/#Example","page":"Linear Model Predictive Control","title":"Example","text":"","category":"section"},{"location":"linearmpc/","page":"Linear Model Predictive Control","title":"Linear Model Predictive Control","text":"Consider a mass string linear system...","category":"page"},{"location":"Examples/#Examples","page":"Examples","title":"Examples","text":"","category":"section"},{"location":"Examples/","page":"Examples","title":"Examples","text":"This section of the documentation describes MPC regulators using EasyMPC for engineering applications.","category":"page"},{"location":"#EasyMPC-a-Model-Predictive-Control-design-facilitor","page":"Home","title":"EasyMPC a Model Predictive Control design facilitor","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"EasyMPC provides Model Predictive Control design for engineering applications.","category":"page"},{"location":"","page":"Home","title":"Home","text":"EasyMPC is in early stage, please consider the package as experimental. ","category":"page"},{"location":"#Installation","page":"Home","title":"Installation","text":"","category":"section"},{"location":"#Acknowledgements","page":"Home","title":"Acknowledgements","text":"","category":"section"},{"location":"","page":"Home","title":"Home","text":"EasyMPC is available thanks to valuable Julia package ecosystem JuMP.jl, OSQP.jl, Controlsystems.jl, LinearAlgebra.jl, ForwardDiff.jl.","category":"page"}]
}