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
solver = optimizer_with_attributes(Gurobi.Optimizer, "OutputFlag" => false, "Method" => 2, "Crossover"=> 0, "BarConvTol" => 1e-10)


for i in 1:10
    println("RUNNING ITERATION $i")
    graph,linking_variables_maps = GenX.generate_graph_model(setup,inputs,solver, solver)
    # Note that this graph is not partitioned into subgraphs. The `BendersAlgorithm` constructore
    # will do this automatically, placing each node on a subgraph, if the user does not define the 
    # subgraphs themselves; this is the approach used for the regularized instance just to highlight
    # both ways of doing the partitioning

    node_membership_vector = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
    partition = Plasmo.Partition(graph, node_membership_vector)
    apply_partition!(graph, partition)

    BendersAlg = BendersAlgorithm(graph, local_subgraphs(graph)[1],
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
    CSV.write((@__DIR__)*"/results/benders_no_regularization_run$i.csv", df)

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
        CSV.write((@__DIR__)*"/results/benders_regularization_run$i.csv", df)
    end

    graph,linking_variables_maps = GenX.generate_graph_model(setup,inputs,solver, solver)

    node_membership_vector = [1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1]
    partition = Plasmo.Partition(graph, node_membership_vector)
    apply_partition!(graph, partition)

    BendersAlg = BendersAlgorithm(graph, local_subgraphs(graph)[1],
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
    CSV.write((@__DIR__)*"/results/benders_larger_root_problem_run$i.csv", df)
end
