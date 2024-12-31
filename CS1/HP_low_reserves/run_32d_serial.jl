using Plasmo
using DelimitedFiles, CSV, DataFrames, JuMP
using Gurobi

############## LOAD IN DATA ###########################

line_data = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/Lines.csv"))
bus_data = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/Buses.csv"))
gen_data = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/All_Generators.csv"))
gen_r = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/Generators_R.csv"))
gen_DA = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/Generators_DAUC.csv"))
gen_ST = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/Generators_STUC.csv"))

gen_data_conv = vcat(gen_DA, gen_ST)

gen_data_DA = vcat(gen_DA, gen_r)

D_DA_no_reserves = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/DA_bus_demand_extrapolation.csv"))
D_ST_no_reserves = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/ST_bus_demand_15min_extrapolation.csv"))
D_RT_no_reserves = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/RT_bus_demand_15min_extrapolation.csv"))

wind_DA = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/Wind/DA_extrapolation.csv"))
wind_ST = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/Wind/ST_15min_extrapolation.csv"))
wind_RT = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/Wind/RT_15min_extrapolation.csv"))

solar_DA = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/Solar/DA_extrapolation.csv"))
solar_ST = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/Solar/ST_15min_extrapolation.csv"))
solar_RT = DataFrame(CSV.File((@__DIR__)*"/../datasets/nrel118/Solar/RT_15min_extrapolation.csv"))

D_DA = copy(D_DA_no_reserves)
D_ST = copy(D_ST_no_reserves)
D_RT = copy(D_RT_no_reserves)

# Add the reserves to the demand values
D_DA[:, 3:end] .= D_DA_no_reserves[:, 3:end] .* 1.1
D_ST[:, 3:end] .= D_ST_no_reserves[:, 3:end] .* 1.1
D_RT[:, 3:end] .= D_RT_no_reserves[:, 3:end] .* 1.025

# Create dictionaries containing the renewable data
xi_DA = Dict()
xi_ST = Dict()
xi_RT = Dict()

wind_names = names(wind_DA)[2:end]
solar_names = names(solar_DA)[2:end]

for i in 1:length(wind_names)
    xi_DA[wind_names[i]] = wind_DA[:, wind_names[i]]
    xi_ST[wind_names[i]] = wind_ST[:, wind_names[i]]
    xi_RT[wind_names[i]] = wind_RT[:, wind_names[i]]
end

for i in 1:length(solar_names)
    xi_DA[solar_names[i]] = solar_DA[:, solar_names[i]]
    xi_ST[solar_names[i]] = solar_ST[:, solar_names[i]]
    xi_RT[solar_names[i]] = solar_RT[:, solar_names[i]]
end

B = line_data[:, "Susceptance"]

# Define costs
phi_c = 25 # curtailment cost
phi_o = 25 # overgeneration cost
phi_u = 5000 # unmet load cost

# Empty data frame for generators committed in the HA level
gen_com_HA = gen_data[gen_data[:, "Fuel"] .== "EMPTY", :]

########################## LOAD IN .jl FILES FOR BUILDING PROBLEM ##########################################

include((@__DIR__)*"/layer_construction.jl")
include((@__DIR__)*"/link_solutions.jl")

statuses = [MOI.INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED]
########################## DEFINE FUNCTION FOR BUILDING 1 DAY PROBLEM #######################

# Define a function for building and solving the graph of the first day
function run_day1_serial(graph_set)

    # Initialize DAUC graph
    graph_DAUC = OptiGraph()

    # Add buses to the graph
    build_bus_over_time(graph_DAUC, 25, D_DA[:, 3:27], xi_DA, 1, gen_DA, gen_data_DA, gen_DA)
    # Add links across time for binary variable constraints and ramping constraints
    link_DA_over_time(graph_DAUC, gen_data_DA)
    # Add constraints
    link_between_DA(graph_DAUC, [graph_DAUC], 1)

    # Set optimizer
    set_optimizer(graph_DAUC, Gurobi.Optimizer)

    # Set MIPgap
    set_optimizer_attribute(graph_DAUC, "MIPGap", .001)
    set_optimizer_attribute(graph_DAUC, "LogFile", (@__DIR__)*"/gurobi_logs/serial/logfile_DA1.log")
    set_optimizer_attribute(graph_DAUC, "TimeLimit", 900)

    # Set objective of graph
    set_to_node_objectives(graph_DAUC)

    # Solve DAUC subproblem
    println("Optimizing DA Problem")
    optimize!(graph_DAUC)

    # Save solutions
    DAUC_sols = [value(graph_DAUC, var) for var in all_variables(graph_DAUC)]
    writedlm((@__DIR__)*"/results/32d_serial/DAUC_results_day1.csv", DAUC_sols, ',')

    # Define full day graph
    graph = OptiGraph()
    push!(graph_set, graph)

    # Add DAUC subproblem to graph
    add_subgraph!(graph, graph_DAUC)
    set_optimizer(graph, Gurobi.Optimizer)

    # Build STUC subproblem graphs
    for i in 1:8
        local graph_STUC = OptiGraph()
        # Add buses to graph
        build_bus_over_time(graph_STUC, 16, D_ST[:, (3 + (i - 1) * 12):(2 + (i - 1) * 12 + 16)], xi_ST, 0.25, gen_ST, gen_data, gen_ST, offset = (i - 1) * 12, add_slacks = true)
        # Link subproblem graph over time
        link_ST_over_time(graph_STUC, add_slacks = true)
        # Link DAUC to STUC problem
        link_DA_sol_to_ST(graph, graph_STUC, (1 + (i - 1) * 3):(4 + (i - 1) * 3), add_slacks = true)

        # Set objective of graph
        set_to_node_objectives(graph_STUC)

        # Set optimizer
        set_optimizer(graph_STUC, Gurobi.Optimizer)
        set_optimizer_attribute(graph_STUC, "MIPGap", .003)
        set_optimizer_attribute(graph_STUC, "LogFile", (@__DIR__)*"/gurobi_logs/serial/logfile_ST1_$(i).log")
        set_optimizer_attribute(graph_STUC, "TimeLimit", 300)

        # Add STUC subgraph to full day graph
        add_subgraph!(graph, graph_STUC)
    end

    # Build HAED subproblem graphs
    for i in 1:96
        local graph_HAED = OptiGraph()
        # Add buses to graph
        build_bus_over_time(graph_HAED, 5, D_RT[:, (3 + (i - 1)):(2 + (i - 1) + 5)], xi_RT, 0.25, gen_com_HA, gen_data, gen_data_conv, offset = (i - 1), HA = true, add_slacks = true)
        # Set objective of graph
        set_to_node_objectives(graph_HAED)
        # Set optimizer
        set_optimizer(graph_HAED, Gurobi.Optimizer)
        # Add HAED subgraph to full day graph
        add_subgraph!(graph, graph_HAED)
    end

    for i in 1:8
        graph_ST = getsubgraphs(graph)[1 + i]
        if i == 1
            ST_sol_time = @elapsed begin
            ST_subgraph_set = getsubgraphs(graph)[2:(i + 1)]

            # for each STUC subgraph, add up/down constraints
            link_between_ST_sol(graph, graph_ST, ST_subgraph_set, 1; day_num = 1, add_slacks = true)

            # Optimize and save solutions
            optimize!(graph_ST)
            if termination_status(graph_ST) in statuses
                relax_slacks(graph_ST, gen_data_conv)
                optimize!(graph_ST)
                print_infeasible("ST", 1, i, graph_ST)
            end
            STUC_sols = [value(graph_ST, var) for var in all_variables(graph_ST)]
            writedlm((@__DIR__)*"/results/32d_serial/STUC_results$(i)_day1.csv", STUC_sols, ',')
            end

            # Solve HAED subproblem graphs
            for j in 1:12
                println()
                println("RUNNING $j")
                println()

                # Link solutions to HAED subproblem
                link_sol_over_HA(graph, j, add_slacks = true)
                HA_graph = getsubgraphs(graph)[9 + j]
                # Optimize and save solutions
                optimize!(HA_graph)
                if termination_status(HA_graph) in statuses
                    relax_slacks(HA_graph, gen_data_conv)
                    optimize!(HA_graph)
                print_infeasible("HA", 1, j, HA_graph)
            end
                HA_sols = [value(HA_graph, var) for var in all_variables(HA_graph)]
                writedlm((@__DIR__)*"/results/32d_serial/HAED_results$(j)_day1.csv", HA_sols, ',')
            end
        else
            ST_sol_time = @elapsed begin
            ST_subgraph_set = getsubgraphs(graph)[2:(i + 1)]

            # for each STUC subgraph, add up/down constraints
            link_between_ST_sol(graph, graph_ST, ST_subgraph_set, i; day_num = 1, add_slacks = true)

            # Optimize and save solutions
            optimize!(graph_ST)
            if termination_status(graph_ST) in statuses
                relax_slacks(graph_ST, gen_data_conv)
                optimize!(graph_ST)
                print_infeasible("ST", 1, i, graph_ST)
            end
            STUC_sols = [value(graph_ST, var) for var in all_variables(graph_ST)]
            writedlm((@__DIR__)*"/results/32d_serial/STUC_results$(i)_day1.csv", STUC_sols, ',')
            end

            # Solve HAED subproblem graphs
            j_range = (1 + (i - 1) * 12):(12 + (i - 1) * 12)
            for j in j_range
                println()
                println("RUNNING $j")
                println()
                # Link solutions to HAED subproblem
                link_sol_over_HA(graph, j, add_slacks = true)
                HA_graph = getsubgraphs(graph)[9 + j]

                # Optimize and save solutions
                optimize!(HA_graph)
                if termination_status(HA_graph) in statuses
                    relax_slacks(HA_graph, gen_data_conv)
                    optimize!(HA_graph)
                print_infeasible("HA", 1, j, HA_graph)
            end
                HA_sols = [value(HA_graph, var) for var in all_variables(HA_graph)]
                writedlm((@__DIR__)*"/results/32d_serial/HAED_results$(j)_day1.csv", HA_sols, ',')
            end
        end
    end
end

# Define a function for building and solving the graph of the second day
function run_day2_serial(graph_set)
    ########################## Solve Day 2 ##################################

    # Initialize DAUC graph
    graph_DAUC = OptiGraph()

    # Add buses to the graph
    build_bus_over_time(graph_DAUC, 25, D_DA[:, 27:51], xi_DA, 1, gen_DA, gen_data_DA, gen_DA, offset = 24)
    # Add links across time for binary variable constraints and ramping constraints
    link_DA_over_time(graph_DAUC, gen_data_DA)

    # build set of DAUC subproblems for previous day
    graph_DAUC_set = []
    push!(graph_DAUC_set, getsubgraphs(graph_set[1])[1])
    push!(graph_DAUC_set, graph_DAUC)

    # Get previuos day HA graph
    HA_subgraph_for_DA = getsubgraphs(graph_set[1])[105]
    # Add constraints
    link_between_DA(graph_DAUC, graph_DAUC_set, 2; HA_graph = HA_subgraph_for_DA)

    # Set optimizer
    set_optimizer(graph_DAUC, Gurobi.Optimizer)

    # Set MIPgap
    set_optimizer_attribute(graph_DAUC, "MIPGap", .001)
    set_optimizer_attribute(graph_DAUC, "LogFile", (@__DIR__)*"/gurobi_logs/serial/logfile_DA2.log")
    set_optimizer_attribute(graph_DAUC, "TimeLimit", 900)

    # Set objective of graph
    set_to_node_objectives(graph_DAUC)

    # Optimize DAUC subproblem
    println("Optimizing DA Problem")
    optimize!(graph_DAUC)

    # Save solutions
    DAUC_sols = [value(graph_DAUC, var) for var in all_variables(graph_DAUC)]
    writedlm((@__DIR__)*"/results/32d_serial/DAUC_results_day2.csv", DAUC_sols, ',')

    # Define full day graph
    graph = OptiGraph()
    push!(graph_set, graph)

    # Add DAUC subproblem to graph
    add_subgraph!(graph, graph_DAUC)
    set_optimizer(graph, Gurobi.Optimizer)

    # Build STUC subproblems
    for i in 1:8
        local graph_STUC = OptiGraph()
        # add buses to graph
        build_bus_over_time(graph_STUC, 16, D_ST[:, (3 + 96 + (i - 1) * 12):(2 + 96 + (i - 1) * 12 + 16)], xi_ST, 0.25, gen_ST, gen_data, gen_ST, offset = (i - 1) * 12 + 96, add_slacks = true)
        # link subproblem graph over time
        link_ST_over_time(graph_STUC, add_slacks = true)
        # link DAUC to STUC subproblem
        link_DA_sol_to_ST(graph, graph_STUC, (1 + (i - 1) * 3):(4 + (i - 1) * 3), add_slacks = true)

        # Set objective of graph
        set_to_node_objectives(graph_STUC)

        # Set optimizer
        set_optimizer(graph_STUC, Gurobi.Optimizer)
        set_optimizer_attribute(graph_STUC, "MIPGap", .003)
        set_optimizer_attribute(graph_STUC, "LogFile", (@__DIR__)*"/gurobi_logs/serial/logfile_ST2_$(i).log")
        set_optimizer_attribute(graph_STUC, "TimeLimit", 300)

        # Add STUC subgraph to full day graph
        add_subgraph!(graph, graph_STUC)
    end

    # Build HAED subproblem graphs
    for i in 1:96
        local graph_HAED = OptiGraph()
        # Add buses to graph
        build_bus_over_time(graph_HAED, 5, D_RT[:, (96 + 3 + (i - 1)):(96 + 2 + (i - 1) + 5)], xi_RT, 0.25, gen_com_HA, gen_data, gen_data_conv, offset = (i - 1 + 96), HA = true, add_slacks = true)
        # Set objective of graph
        set_to_node_objectives(graph_HAED)
        # Set optimizer
        set_optimizer(graph_HAED, Gurobi.Optimizer)
        # Add HAED subgraph to full day graph
        add_subgraph!(graph, graph_HAED)
    end


    for i in 1:8
        graph_ST = getsubgraphs(graph)[1 + i]

        if i == 1
            subgraph_set1 = getsubgraphs(graph_set[1])[7:9]
            subgraph_set2 = getsubgraphs(graph)[2:(i + 1)]
            ST_subgraph_set = vcat(subgraph_set1, subgraph_set2)
            # for each STUC subgraph, add up/down constraints
            link_between_ST_sol(graph, graph_ST, ST_subgraph_set, i; day_num = 2, graph_last = graph_set[1], add_slacks = true)

            # Optimize and save solutions
            optimize!(graph_ST)
            if termination_status(graph_ST) in statuses
                relax_slacks(graph_ST, gen_data_conv)
                optimize!(graph_ST)
                print_infeasible("ST", 2, i, graph_ST)
            end
            STUC_sols = [value(graph_ST, var) for var in all_variables(graph_ST)]
            writedlm((@__DIR__)*"/results/32d_serial/STUC_results$(i)_day2.csv", STUC_sols, ',')

            # Link ST solutions to HAED graph
            link_sol_over_HA_new_day(graph, graph_set[1], monolithic = false, add_slacks = true)
        elseif i == 2
            subgraph_set1 = getsubgraphs(graph_set[1])[8:9]
            subgraph_set2 = getsubgraphs(graph)[2:(i + 1)]
            ST_subgraph_set = vcat(subgraph_set1, subgraph_set2)
            # for each STUC subgraph, add up/down constraints
            link_between_ST_sol(graph, graph_ST, ST_subgraph_set, i; day_num = 2, graph_last = graph_set[1], add_slacks = true)
            # Optimize and save solutions
            optimize!(graph_ST)
            if termination_status(graph_ST) in statuses
                relax_slacks(graph_ST, gen_data_conv)
                optimize!(graph_ST)
                print_infeasible("ST", 2, i, graph_ST)
            end
            STUC_sols = [value(graph_ST, var) for var in all_variables(graph_ST)]
            writedlm((@__DIR__)*"/results/32d_serial/STUC_results$(i)_day2.csv", STUC_sols, ',')
        elseif i == 3
            subgraph_set1 = getsubgraphs(graph_set[1])[9:9]
            subgraph_set2 = getsubgraphs(graph)[2:(i + 1)]
            ST_subgraph_set = vcat(subgraph_set1, subgraph_set2)
            # for each STUC subgraph, add up/down constraints
            link_between_ST_sol(graph, graph_ST, ST_subgraph_set, i; day_num = 2, graph_last = graph_set[1], add_slacks = true)
            # Optimize and save solutions
            optimize!(graph_ST)
            if termination_status(graph_ST) in statuses
                relax_slacks(graph_ST, gen_data_conv)
                optimize!(graph_ST)
                print_infeasible("ST", 2, i, graph_ST)
            end
            STUC_sols = [value(graph_ST, var) for var in all_variables(graph_ST)]
            writedlm((@__DIR__)*"/results/32d_serial/STUC_results$(i)_day2.csv", STUC_sols, ',')
        else
            ST_subgraph_set = getsubgraphs(graph)[2:(i + 1)]
            # for each STUC subgraph, add up/down constraints
            link_between_ST_sol(graph, graph_ST, ST_subgraph_set, i; day_num = 2, graph_last = graph_set[1], add_slacks = true)
            # Optimize and save solutions
            optimize!(graph_ST)
            if termination_status(graph_ST) in statuses
                relax_slacks(graph_ST, gen_data_conv)
                optimize!(graph_ST)
                print_infeasible("ST", 2, i, graph_ST)
            end
            STUC_sols = [value(graph_ST, var) for var in all_variables(graph_ST)]
            writedlm((@__DIR__)*"/results/32d_serial/STUC_results$(i)_day2.csv", STUC_sols, ',')
        end
        j_range = (1 + (i - 1) * 12):(12 + (i - 1) * 12)
        for j in j_range
            println()
            println("RUNNING $j")
            println()
            # Link solutions to HAED subproblem
            link_sol_over_HA(graph, j, add_slacks = true)
            HA_graph = getsubgraphs(graph)[9 + j]
            # Optimize and save solutions
            optimize!(HA_graph)
            if termination_status(HA_graph) in statuses
                relax_slacks(HA_graph, gen_data_conv)
                optimize!(HA_graph)
                print_infeasible("HA", 2, j, HA_graph)
            end
            HA_sols = [value(HA_graph, var) for var in all_variables(HA_graph)]
            writedlm((@__DIR__)*"/results/32d_serial/HAED_results$(j)_day2.csv", HA_sols, ',')
        end
    end
end

# Define a function for building and solving the graph of the ith day
function run_dayi_serial(graph_set, day)
    # Remove the last graph of the graph_set (expensive to store too many in memory)
    if length(graph_set) >= 3
        popfirst!(graph_set)
    end
    ########################## Solve Day i ##################################

    # Initialize DAUC graph
    graph_DAUC = OptiGraph()

    DA_range = (3 + 24 * (day - 1)):(3 + 24 * day)
    # Add buses to graph
    build_bus_over_time(graph_DAUC, 25, D_DA[:, DA_range], xi_DA, 1, gen_DA, gen_data_DA, gen_DA, offset = (day - 1) * 24)
    # Add links across time for binary variable constraints and ramping constraints
    link_DA_over_time(graph_DAUC, gen_data_DA)

    # Build set of DAUC subproblems for previous day
    graph_DAUC_set = []
    for k in 1:length(graph_set)
        push!(graph_DAUC_set, getsubgraphs(graph_set[k])[1])
    end
    push!(graph_DAUC_set, graph_DAUC)

    HA_subgraph_for_DA = getsubgraphs(graph_set[length(graph_set)])[105]
    # Add constraints
    link_between_DA(graph_DAUC, graph_DAUC_set, day; HA_graph = HA_subgraph_for_DA)

    # Set optimizer
    set_optimizer(graph_DAUC, Gurobi.Optimizer)

    # Set MIPgap
    #if day == 27 || day == 3 || day == 31
    #    set_optimizer_attribute(graph_DAUC, "MIPGap", .0005)
    #else
    #if (day == 14) || (day == 31)
    #    set_optimizer_attribute(graph_DAUC, "MIPGap", .0005)
    #else
    #    set_optimizer_attribute(graph_DAUC, "MIPGap", .001)
    #end

    set_optimizer_attribute(graph_DAUC, "MIPGap", .001)
    set_optimizer_attribute(graph_DAUC, "Threads", Threads.nthreads())
    set_optimizer_attribute(graph_DAUC, "LogFile", (@__DIR__)*"/gurobi_logs/serial/logfile_DA_$(day).log")
    set_optimizer_attribute(graph_DAUC, "TimeLimit", 900)

    # Set objective of graph
    set_to_node_objectives(graph_DAUC)

    # Optimize DAUC subproblem
    println("Optimizing DA Problem")
    optimize!(graph_DAUC)

    # Save solutions
    DAUC_sols = [value(graph_DAUC, var) for var in all_variables(graph_DAUC)]
    writedlm((@__DIR__)*"/results/32d_serial/DAUC_results_day$(day).csv", DAUC_sols, ',')

    # Define full day graph
    graph = OptiGraph()
    push!(graph_set, graph)

    # Add DAUC subproblem to graph
    add_subgraph!(graph, graph_DAUC)
    set_optimizer(graph, Gurobi.Optimizer)

    # Build STUC subproblems
    for i in 1:8
        local graph_STUC = OptiGraph()
        # add buses to graph
        ST_range = (3 + (day - 1) * 96 + (i - 1) * 12):(2 + (day - 1) * 96 + (i - 1) * 12 + 16)
        build_bus_over_time(graph_STUC, 16, D_ST[:, ST_range], xi_ST, 0.25, gen_ST, gen_data, gen_ST, offset = (day - 1) * 96 + (i - 1) * 12, add_slacks = true)
        # link subproblem graph over time
        link_ST_over_time(graph_STUC, add_slacks = true)
        # link DAUC to STUC subproblem
        link_DA_sol_to_ST(graph, graph_STUC, (1 + (i - 1) * 3):(4 + (i - 1) * 3), add_slacks = true)
        # Set optimizer
        set_optimizer(graph_STUC, Gurobi.Optimizer)

        # Set objective of graph
        set_to_node_objectives(graph_STUC)

        # Set MIPgap
        #if ((day == 26) && (i != 6)) || (day == 3) || ((day == 31) && (i == 2))
        #    set_optimizer_attribute(graph_STUC, "MIPGap", .001)
        #elseif day == 27
        #    set_optimizer_attribute(graph_STUC, "MIPGap", .0005)
        #elseif ((day == 27) && (i == 6)) || ((day == 31) && (i == 1))
        #    set_optimizer_attribute(graph_STUC, "MIPGap", .004)
        #else
        set_optimizer_attribute(graph_STUC, "MIPGap", .003)
        #end
        set_optimizer_attribute(graph_STUC, "Threads", Threads.nthreads())
        set_optimizer_attribute(graph_STUC, "TimeLimit", 300)
        set_optimizer_attribute(graph_STUC, "LogFile", (@__DIR__)*"/gurobi_logs/serial/logfile_ST$(day)_$(i).log")

        # Add STUC subgraph to full day graph
        add_subgraph!(graph, graph_STUC)
    end

    # Build HAED subproblem graphs
    for i in 1:96
        local graph_HAED = OptiGraph()
        HA_range = ((day - 1) * 96 + 3 + (i - 1)):((day - 1) * 96 + 2 + (i - 1) + 5)
        # Add buses to graph
        build_bus_over_time(graph_HAED, 5, D_RT[:, HA_range], xi_RT, 0.25, gen_com_HA, gen_data, gen_data_conv, offset = ((day - 1) * 96 + (i - 1)), HA = true, add_slacks = true)
        # Set objective of graph
        set_to_node_objectives(graph_HAED)
        # Set optimizer
        set_optimizer(graph_HAED, Gurobi.Optimizer)
        # Add HAED subgraph to full day graph
        add_subgraph!(graph, graph_HAED)
    end


    for i in 1:8
        graph_ST = getsubgraphs(graph)[1 + i]

        if i == 1
            subgraph_set1 = getsubgraphs(graph_set[2])[7:9]
            subgraph_set2 = getsubgraphs(graph)[2:(i + 1)]
            ST_subgraph_set = vcat(subgraph_set1, subgraph_set2)
            # for each STUC subgraph, add up/down constraints
            link_between_ST_sol(graph, graph_ST, ST_subgraph_set, i; day_num = day, graph_last = graph_set[2], add_slacks = true)
            # Optimize and save solutions
            optimize!(graph_ST)
            if termination_status(graph_ST) in statuses
                relax_slacks(graph_ST, gen_data_conv)
                optimize!(graph_ST)
                print_infeasible("ST", day, i, graph_ST)
            end
            STUC_sols = [value(graph_ST, var) for var in all_variables(graph_ST)]
            writedlm((@__DIR__)*"/results/32d_serial/STUC_results$(i)_day$(day).csv", STUC_sols, ',')

            # Link ST solutions to HAED graph
            link_sol_over_HA_new_day(graph, graph_set[2], monolithic = false, add_slacks = true)
        elseif i == 2
            subgraph_set1 = getsubgraphs(graph_set[2])[8:9]
            subgraph_set2 = getsubgraphs(graph)[2:(i + 1)]
            ST_subgraph_set = vcat(subgraph_set1, subgraph_set2)
            # for each STUC subgraph, add up/down constraints
            link_between_ST_sol(graph, graph_ST, ST_subgraph_set, i; day_num = day, graph_last = graph_set[2], add_slacks = true)
            # Optimize and save solutions
            optimize!(graph_ST)
            if termination_status(graph_ST) in statuses
                relax_slacks(graph_ST, gen_data_conv)
                optimize!(graph_ST)
                print_infeasible("ST", day, i, graph_ST)
            end
            STUC_sols = [value(graph_ST, var) for var in all_variables(graph_ST)]
            writedlm((@__DIR__)*"/results/32d_serial/STUC_results$(i)_day$(day).csv", STUC_sols, ',')
        elseif i == 3
            subgraph_set1 = getsubgraphs(graph_set[2])[9:9]
            subgraph_set2 = getsubgraphs(graph)[2:(i + 1)]
            ST_subgraph_set = vcat(subgraph_set1, subgraph_set2)
            # for each STUC subgraph, add up/down constraints
            link_between_ST_sol(graph, graph_ST, ST_subgraph_set, i; day_num = day, graph_last = graph_set[2], add_slacks = true)
            optimize!(graph_ST)
            if termination_status(graph_ST) in statuses
                relax_slacks(graph_ST, gen_data_conv)
                optimize!(graph_ST)
                print_infeasible("ST", day, i, graph_ST)
            end
            STUC_sols = [value(graph_ST, var) for var in all_variables(graph_ST)]
            writedlm((@__DIR__)*"/results/32d_serial/STUC_results$(i)_day$(day).csv", STUC_sols, ',')
        else
            ST_subgraph_set = getsubgraphs(graph)[2:(i + 1)]
            # for each STUC subgraph, add up/down constraints
            link_between_ST_sol(graph, graph_ST, ST_subgraph_set, i; day_num = day, graph_last = graph_set[2], add_slacks = true)
            # Optimize and save solutions
            optimize!(graph_ST)
            if termination_status(graph_ST) in statuses
                relax_slacks(graph_ST, gen_data_conv)
                optimize!(graph_ST)
                print_infeasible("ST", day, i, graph_ST)
            end
            STUC_sols = [value(graph_ST, var) for var in all_variables(graph_ST)]
            writedlm((@__DIR__)*"/results/32d_serial/STUC_results$(i)_day$(day).csv", STUC_sols, ',')
        end
        j_range = (1 + (i - 1) * 12):(12 + (i - 1) * 12)
        for j in j_range
            println()
            println("RUNNING $j")
            println()
            # Link solutions to HAED subproblem
            link_sol_over_HA(graph, j, add_slacks = true)
            HA_graph = getsubgraphs(graph)[9 + j]
            # Optimize and save solutions
            optimize!(HA_graph)
            if termination_status(HA_graph) in statuses
                relax_slacks(HA_graph, gen_data_conv)
                optimize!(HA_graph)
                print_infeasible("HA", day, j, HA_graph)
            end
            HA_sols = [value(HA_graph, var) for var in all_variables(HA_graph)]
            writedlm((@__DIR__)*"/results/32d_serial/HAED_results$(j)_day$(day).csv", HA_sols, ',')
        end
    end
end

graph_set = []
run_times = []
# Solve day 1
d1_time = @elapsed run_day1_serial(graph_set)
push!(run_times, d1_time)

GC.gc()
# Solve day 2
d2_time = @elapsed run_day2_serial(graph_set)
push!(run_times, d2_time)
GC.gc()

# Solve day 3-32
for i in 3:32
    di_time = @elapsed run_dayi_serial(graph_set, i)
    push!(run_times, di_time)
    GC.gc()
end

println(sum(run_times) / 60, " min")

writedlm((@__DIR__)*"/results/serial_run_times.csv", run_times)
