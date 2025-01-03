case = dirname(@__FILE__)

# Load required pacakges
using GenX,Revise
using PlasmoBenders, JuMP, Plasmo, Gurobi, Suppressor, HiGHS
using DataFrames, CSV

# Load in settings; this is done through the yaml files included in this directory
genx_settings = GenX.get_settings_path(case, "genx_settings.yml") # Settings YAML file path
writeoutput_settings = GenX.get_settings_path(case, "output_settings.yml") # Write-output settings YAML file path
setup = GenX.configure_settings(genx_settings, writeoutput_settings) # mysetup dictionary stores settings and GenX-specific parameters

setup["Benders"] = 1;
benders_settings_path = GenX.get_settings_path(case, "benders_settings.yml")
setup_benders = GenX.configure_benders(benders_settings_path)
setup = merge(setup,setup_benders);
setup["settings_path"] = GenX.get_settings_path(case);

# Load in inputs
inputs = GenX.load_inputs(setup, case);

# Set subproblem solver
solver = optimizer_with_attributes(Gurobi.Optimizer, "OutputFlag" => false, "Method" => 2, "Crossover"=> 0)

#PLANNING_OPTIMIZER = GenX.configure_benders_planning_solver(setup["settings_path"]);
#SUBPROB_OPTIMIZER =  GenX.configure_benders_subprob_solver(setup["settings_path"]);
graph,linking_variables_maps = GenX.generate_graph_model(setup,inputs,solver, solver)

#graph,linking_variables_maps = GenX.generate_graph_model(setup,inputs,PLANNING_OPTIMIZER,SUBPROB_OPTIMIZER)


set_to_node_objectives(graph)
set_optimizer(graph, Gurobi.Optimizer)
#optimize!(graph)

BendersAlg = BendersAlgorithm(graph, local_nodes(graph)[1],
    regularize = false,
    multicut = true,
    parallelize_benders = true,
    tol = 1e-4,
    solver=solver,
    max_iters = 350,
    warm_start = false,
)
run_algorithm!(BendersAlg)

df = DataFrame()
df[!, "UB"] = BendersAlg.upper_bounds
df[!, "LB"] = BendersAlg.lower_bounds
df[!, "Time"] = BendersAlg.time_iterations
CSV.write((@__DIR__)*"/benders_no_regularization.csv", df)


graph,linking_variables_maps = GenX.generate_graph_model(setup,inputs,solver, solver)

@suppress_err begin
    BendersAlg = BendersAlgorithm(graph, local_nodes(graph)[1],
        regularize = true,
        multicut = true,
        parallelize_benders = true,
        tol = 1e-4,
        solver=solver,
        max_iters = 350,
        warm_start = false,
    )
    run_algorithm!(BendersAlg)

    df = DataFrame()
    df[!, "UB"] = BendersAlg.upper_bounds
    df[!, "LB"] = BendersAlg.lower_bounds
    df[!, "Time"] = BendersAlg.time_iterations
    CSV.write((@__DIR__)*"/benders_regularization.csv", df)
end
