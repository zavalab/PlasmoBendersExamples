using Plasmo, JuMP

function copy_var_attributes(JuMP_var, var)
    if is_binary(JuMP_var)
        set_binary(var)
    elseif is_integer(JuMP_var)
        set_integer(var)
    end

    if has_lower_bound(JuMP_var)
        set_lower_bound(var, lower_bound(JuMP_var))
    end
    if has_upper_bound(JuMP_var)
        set_upper_bound(var, upper_bound(JuMP_var))
    end

    if has_start_value(JuMP_var)
        set_start_value(var, start_value(JuMP_var))
    end

    if is_fixed(JuMP_var)
        fix(var, fix_value(JuMP_var); force = true)
    end
end

function add_new_var!(node, JuMP_var, var_to_var, node_to_var)
    new_var = @variable(node)
    var_to_var[JuMP_var] = new_var
    push!(node_to_var[node], new_var)

    copy_var_attributes(JuMP_var, new_var)
end

function get_expr(con_obj, var_to_var, node_to_var)
    vars = con_obj.func.terms.keys

    if all(x -> x in keys(var_to_var), vars)
        node_set = unique([JuMP.owner_model(var_to_var[var]) for var in vars if (var in keys(var_to_var))])
        owning_node = node_set[1]

        if length(node_set) > 1
            link = true
        elseif length(node_set) == 1
            link = false
        else
            error("variables don't have a node!")
        end

        new_expr = sum(var_to_var[var] * con_obj.func.terms[var] for var in vars)
    elseif any(x -> x in keys(var_to_var), vars)
        new_vars = [i for i in vars if !(i in keys(var_to_var))]
        node_set = unique([JuMP.owner_model(var_to_var[var]) for var in vars if (var in keys(var_to_var))])
        owning_node = node_set[1]

        if length(node_set) > 1
            link = true
        elseif length(node_set) == 1
            link = false
        else
            error("variables don't have a node!")
        end

        for new_var in new_vars
            add_new_var!(owning_node, new_var, var_to_var, node_to_var)
        end

        new_expr = sum(var_to_var[var] * con_obj.func.terms[var] for var in vars)
    else
        error("Constraint $con_obj has no node :(")
    end

    return new_expr, link, owning_node
end

function build_graph_from_model(model, sys, horizon)
    buses = collect(get_components(Bus, sys))
    arcs = collect(get_components(Arc, sys))
    lines = collect(get_components(Line, sys))
    gens  = collect(get_components(Generator, sys))
    tts   = collect(get_components(TapTransformer, sys))
    bats = vcat(collect(get_components(BatteryEMS, sys)), collect(get_components(GenericBattery, sys)))

    used_arcs = []

    for i in 1:length(lines)
        push!(used_arcs, lines[i].arc)
    end
    for i in 1:length(tts)
        push!(used_arcs, tts[i].arc)
    end

    used_arcs = unique(used_arcs)

    bus_names = [buses[i].name for i in 1:length(buses)]
    bus_numbers = [buses[i].number for i in 1:length(buses)]
    arc_names = [(used_arcs[i].from.name, used_arcs[i].to.name) for i in 1:length(used_arcs)]
    gen_names = [gens[i].name for i in 1:length(gens)]
    line_names = [lines[i].name for i in 1:length(lines)]
    tt_names = [tts[i].name for i in 1:length(tts)]
    bat_names = [bats[i].name for i in 1:length(bats)]

    graph = OptiGraph()
    @optinode(graph, bus[bus_names, 1:horizon])
    @optinode(graph, arc[arc_names, 1:horizon])

    bus_name_to_number = Dict()
    bus_number_to_name = Dict()

    bus_to_node = Dict()
    arc_to_node = Dict()
    gen_to_node = Dict()
    line_to_node = Dict()
    tt_to_node = Dict()
    bat_to_node = Dict()

    gen_name_to_bus  = Dict()
    line_name_to_arc = Dict()
    tt_name_to_arc = Dict()
    bat_name_to_bus = Dict()

    name_to_node = Dict()

    for i in 1:length(buses)
        bus_to_node[bus_names[i]] = graph[:bus][bus_names[i], :]

        bus_name_to_number[bus_names[i]] = bus_numbers[i]
        bus_number_to_name[bus_numbers[i]] = bus_names[i]
    end

    for i in 1:length(arc_names)
        arc_name = (used_arcs[i].from.name, used_arcs[i].to.name)
        arc_to_node[arc_name] = graph[:arc][arc_names[i], :]
        name_to_node[arc_name] = graph[:arc][arc_names[i], :]
    end

    for i in 1:length(gens)
        gen_name_to_bus[gens[i].name] = gens[i].bus.name
        gen_to_node[gens[i].name] = bus_to_node[gens[i].bus.name]
        name_to_node[gens[i].name] = bus_to_node[gens[i].bus.name]
    end

    for i in 1:length(lines)
        line_arc = lines[i].arc
        arc_name = (line_arc.from.name, line_arc.to.name)
        line_name_to_arc[lines[i].name] = arc_name
        line_to_node[lines[i].name] = arc_to_node[arc_name]
        name_to_node[lines[i].name] = arc_to_node[arc_name]
    end

    for i in 1:length(tts)
        tt_arc = tts[i].arc
        arc_name = (tt_arc.from.name, tt_arc.to.name)
        tt_name_to_arc[tts[i].name] = arc_name
        tt_to_node[tts[i].name] = arc_to_node[arc_name]
        name_to_node[tts[i].name] = arc_to_node[arc_name]
    end

    for i in 1:length(bats)
        bat_name_to_bus[bats[i].name] = bats[i].bus.name
        bat_to_node[bats[i].name] = bus_to_node[bats[i].bus.name]
        name_to_node[bats[i].name] = bus_to_node[bats[i].bus.name]
    end

    variables = model.internal.container.variables
    var_sets = collect(keys(variables))
    constraints = model.internal.container.constraints
    con_sets = collect(keys(constraints))

    node_to_var = Dict{Plasmo.OptiNode, Vector{Any}}()

    for i in 1:length(bus_names)
        for j in 1:horizon
            optinode = graph[:bus][bus_names[i], j]
            node_to_var[optinode] = Any[]
        end
    end
    for i in 1:length(arc_names)
        for j in 1:horizon
            optinode = graph[:arc][arc_names[i], j]
            node_to_var[optinode] = Any[]
        end
    end

    function match_data_to_node(data, axis1, axis2, node_to_var = node_to_var)
        if eltype(axis1) == String
            for (i, name) in enumerate(axis1)
                for (j, time) in enumerate(axis2)
                    var = data[i, j]
                    push!(node_to_var[name_to_node[name][j]], var)
                end
            end
            # Test if it is in the lines, tts, or gens
        elseif eltype(axis1) == Int64
            for (i, num) in enumerate(axis1)
                for (j, time) in enumerate(axis2)
                    var = data[i, j]
                    push!(node_to_var[bus_to_node[bus_number_to_name[num]][j]], var)
                end
            end
        else
            error("Type is incorrect")
        end
    end

    for (i, set) in enumerate(var_sets)
        var_set = variables[var_sets[i]]
        if typeof(var_set) <: JuMP.Containers.DenseAxisArray
            if length(var_set.axes) == 2
                axis1 = var_set.axes[1]
                axis2 = var_set.axes[2]
                data = var_set.data
                match_data_to_node(data, axis1, axis2)
            elseif length(var_set.axes) == 1
                continue
            else
                error("variable set is the wrong length")
            end
            #println(i, "  DENSE")
        elseif typeof(var_set) <: JuMP.Containers.SparseAxisArray
            #println(i, "    SPARSE")
            data = var_set.data
            for key in keys(data)
                name = key[1]
                time = key[3]
                var = data[key]

                push!(node_to_var[name_to_node[name][time]], var)
            end
        else
            println(i, " did not enter")
            println(typeof(var_set))
        end
    end

    var_to_var = Dict{VariableRef, Any}()
    for node in keys(node_to_var)
        var_list = node_to_var[node]
        var_len = length(var_list)
        @variable(node, var_copy[1:var_len])

        for (j, var) in enumerate(var_copy)
            JuMP_var = var_list[j]
            var_to_var[JuMP_var] = var

            copy_var_attributes(JuMP_var, var)
        end
    end

    all_cons = all_constraints(model.internal.container.JuMPmodel, include_variable_in_set_constraints = false)

    con_to_con = Dict()
    graph_con_to_con = Dict()

    for (i, con) in enumerate(all_cons)
        con_obj = constraint_object(con)
        if typeof(con_obj.set) == MOI.EqualTo{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var, node_to_var)
            if link
                graph_con = @linkconstraint(graph, new_expr == con_obj.set.value)
            else
                graph_con = @constraint(owning_node, new_expr == con_obj.set.value)
            end
        elseif typeof(con_obj.set) == MOI.LessThan{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var, node_to_var)
            if link
                graph_con = @linkconstraint(graph, new_expr <= con_obj.set.upper)
            else
                graph_con = @constraint(owning_node, new_expr <= con_obj.set.upper)
            end
        elseif typeof(con_obj.set) == MOI.GreaterThan{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var, node_to_var)
            if link
                graph_con = @linkconstraint(graph, new_expr >= con_obj.set.lower)
            else
                graph_con = @constraint(owning_node, new_expr >= con_obj.set.lower)
            end
        else
            error("constraint object does not match types")
        end
        con_to_con[con] = graph_con
        graph_con_to_con[graph_con] = con
    end

    jm_obj_func = objective_function(model.internal.container.JuMPmodel)

    for (i, node) in enumerate(keys(node_to_var))
        node_vars = intersect(node_to_var[node], jm_obj_func.terms.keys)
        if length(node_vars) > 0
            new_obj_expr = sum(var_to_var[var] * jm_obj_func.terms[var] for var in node_vars)
            @objective(node, Min, new_obj_expr)
        end
    end

    return graph, var_to_var, bus_names, used_arcs
end
#=
graph, var_to_var, bus_names, arc_names = build_graph_from_model(model, sys, horizon)

cg, cm = getcliquegraph(graph)

g = cg.graph

using LightGraphs, SparseArrays
function get_edge_list(g)
    am = LightGraphs.adjacency_matrix(g)
    I_g, J_g, z_g = findnz(am)
    edge_list = Matrix{Int}(zeros(length(I_g), 2))
    for i in 1:length(I_g)
        if J_g[i] <= I_g[i]
            edge_list[i, :] .= [J_g[i], I_g[i]]
        end
    end
    edge_list = edge_list[(edge_list[:, 1] .!= 0), :]
    df = DataFrame(["Source" => edge_list[:, 1], "Target" => edge_list[:, 2]])
    return df
end

df_edges = get_edge_list(g)

node_id = [i for i in 1:length(getnodes(graph))]
node_time = zeros(Int, length(node_id))
node_type = String[]
for i in 1:horizon
    for j in 1:length(bus_names)
        index = j + (i - 1) * (length(bus_names))
        node_time[index] = i
        push!(node_type, "Bus")
    end
end

for i in 1:horizon
    for j in 1:length(arc_names)
        index = (length(bus_names) * horizon) + j + (i - 1) * (length(arc_names))
        node_time[index] = i
        push!(node_type, "Arc")
    end
end

node_time_part = ceil.(node_time ./ 12)

df_nodes = DataFrame(["id" => node_id, "TimePoint" => node_time, "TimeAgg" => node_time_part, "Type" => node_type])

CSV.write((@__DIR__)*"/node_data_RTS48_CP.csv", df_nodes)
CSV.write((@__DIR__)*"/edge_data_RTS48_CP.csv", df_edges)

hypergraph, hypermap = hyper_graph(graph)

partition = Partition(hypergraph, node_time, hypermap)

apply_partition!(graph, partition)

graph1 = getsubgraphs(graph)[1]

cg1, cm1 = getcliquegraph(graph1)

df_edges1 = get_edge_list(cg1.graph)

CSV.write((@__DIR__)*"/edge_data_RTS1_CP.csv", df_edges1)

#TODO: Add typing to speed up code

=#
