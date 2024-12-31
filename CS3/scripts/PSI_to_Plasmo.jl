using Plasmo, JuMP

function _get_variable_node(i, name, av, nodes, var_numbers, node_to_var, var_to_node)
    if name == ""
        return nothing
    else
        if isnumeric(name[1])
            index = 1
            while true
                if isnumeric(name[index + 1])
                    index += 1
                else
                    break
                end
            end

            numbers = parse(Int, name[1:index])

            var_numbers[numbers] += 1
            push!(node_to_var[nodes[numbers]], av[i])
            var_to_node[av[i]] = nodes[numbers]
        else
            last_vals = last(name, 2)
            index = 1
            while true
                last_vals = last(name, index + 1)
                if isnumeric(last_vals[1])
                    index += 1
                else
                    break
                end
            end

            numbers = parse(Int, last_vals[2:index])

            var_numbers[numbers] += 1
            push!(node_to_var[nodes[numbers]], av[i])
            var_to_node[av[i]] = nodes[numbers]
        end
    end
end

function PSI_to_Plasmo(model, horizon = 48)
    jm = model.internal.container.JuMPmodel
    av = all_variables(jm)

    names = name.(av)

    graph = OptiGraph()

    @optinode(graph, n[1:horizon])

    var_numbers = zeros(length(n))

    node_to_var = Dict()
    var_to_node = Dict()
    var_to_var = Dict()

    for i in 1:horizon
        node_to_var[n[i]] = []
    end


    model_vars = model.internal.container.variables
    var_keys = collect(keys(model_vars))

    for i in 1:length(var_keys)
        var_set = model_vars[var_keys[i]]
        if typeof(var_set) <: JuMP.Containers.DenseAxisArray
            axes = var_set.axes

            if length(axes) == 2
                for j in axes[1]
                    for k in axes[2]
                        var_numbers[k] += 1
                        push!(node_to_var[n[k]], var_set[j, k])
                        var_to_node[var_set[j, k]] = n[k]
                    end
                end
            else
                for k in axes[1]
                    var_numbers[k] += 1
                    push!(node_to_var[n[k]], var_set[k])
                    var_to_node[var_set[k]] = n[k]
                end
            end
        else
            dict_data = var_set.data
            for key in keys(dict_data)
                new_var = dict_data[key]
                idx = key[3]
                var_numbers[idx] += 1
                push!(node_to_var[n[idx]], new_var)
                var_to_node[new_var] = n[idx]
            end
        end
    end

    for (i, node) in enumerate(n)
        var_list = node_to_var[node]
        var_len = length(var_list)
        @variable(node, var_copy[1:var_len])

        for (j, var) in enumerate(var_copy)
            JuMP_var = var_list[j]
            var_to_var[JuMP_var] = var

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
    end

    all_cons = all_constraints(jm, include_variable_in_set_constraints = false)

    #co_types = unique(typeof.(constraint_object.(all_cons)))

    function add_new_var!(node, JuMP_var, var_to_var)
        new_var = @variable(node)
        var_to_var[JuMP_var] = new_var
        push!(node_to_var[node], new_var)
        var_to_node[new_var] = node

        if is_binary(JuMP_var)
            set_binary(new_var)
        elseif is_integer(JuMP_var)
            set_integer(new_var)
        end

        if has_lower_bound(JuMP_var)
            set_lower_bound(new_var, lower_bound(JuMP_var))
        end
        if has_upper_bound(JuMP_var)
            set_upper_bound(new_var, upper_bound(JuMP_var))
        end

        if has_start_value(JuMP_var)
            set_start_value(new_var, start_value(JuMP_var))
        end

        if is_fixed(JuMP_var)
            fix(new_var, fix_value(JuMP_var); force = true)
        end
    end

    function get_expr(con_obj, var_to_var)
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
            #new_expr = GenericAffExpr{Float64, Plasmo.NodeVariableRef}()
            #for var in vars
            #    add_to_expression!(new_expr, var_to_var[var], con_obj.func.terms[var])
            #end
        else
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
                add_new_var!(owning_node, new_var, var_to_var)
            end

            new_expr = sum(var_to_var[var] * con_obj.func.terms[var] for var in vars)
            #new_expr = GenericAffExpr{Float64, Plasmo.NodeVariableRef}()
            #for var in vars
            #    add_to_expression!(new_expr, var_to_var[var], con_obj.func.terms[var])
            #end
        end

        return new_expr, link, owning_node
    end

    con_to_con = Dict()
    graph_con_to_con = Dict()

    println("ENTERING LOOP")
    for (i, con) in enumerate(all_cons)
        if i%50000 == 0
            println(i, "  of  ", length(all_cons))
        end
        con_obj = constraint_object(con)
        if typeof(con_obj.set) == MOI.EqualTo{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var)
            if link
                graph_con = @linkconstraint(graph, new_expr == con_obj.set.value)
            else
                graph_con = @constraint(owning_node, new_expr == con_obj.set.value)
            end
        elseif typeof(con_obj.set) == MOI.LessThan{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var)
            if link
                graph_con = @linkconstraint(graph, new_expr <= con_obj.set.upper)
            else
                graph_con = @constraint(owning_node, new_expr <= con_obj.set.upper)
            end
        elseif typeof(con_obj.set) == MOI.GreaterThan{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var)
            if link
                graph_con = @linkconstraint(graph, new_expr >= con_obj.set.lower)
            else
                graph_con = @constraint(owning_node, new_expr >= con_obj.set.lower)
            end
        elseif typeof(con_obj.set) == MOI.Interval{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var)
            if link
                graph_con = @linkconstraint(graph, con_obj.set.lower <= new_expr <= con_obj.set.upper)
            else
                graph_con = @constraint(owning_node, con_obj.set.lower <= new_expr <= con_obj.set.upper)
            end
        else
            println(i)
            println(con)
            println(typeof(con))
            println(typeof(con_obj))
            error("constraint object does not match types")
        end
        con_to_con[con] = graph_con
        graph_con_to_con[graph_con] = con
    end

    #TODO: Ensure that obj_func is aff not quad
    jm_obj_func = objective_function(jm)

    for (i, node) in enumerate(n)
        node_vars = intersect(node_to_var[node], jm_obj_func.terms.keys)
        new_obj_expr = sum(var_to_var[var] * jm_obj_func.terms[var] for var in node_vars)
        @objective(node, Min, new_obj_expr)
    end

    #new_obj_expr = sum(var_to_var[var] * jm_obj_func.terms[var] for var in keys(jm_obj_func.terms))

    #set_objective_function(graph, new_obj_expr)

    set_optimizer(graph, HiGHS.Optimizer)

    return graph, var_to_var, var_to_node, node_to_var, con_to_con
end

function PSI_to_Plasmo_relaxed(model, horizon = 48)
    jm = model.internal.container.JuMPmodel
    av = all_variables(jm)

    names = name.(av)

    graph = OptiGraph()

    @optinode(graph, n[1:horizon])

    var_numbers = zeros(length(n))

    node_to_var = Dict()
    var_to_node = Dict()
    var_to_var = Dict()

    for i in 1:horizon
        node_to_var[n[i]] = []
    end

    for (i, name) in enumerate(names)
        _get_variable_node(i, name, av, n, var_numbers, node_to_var, var_to_node)
    end


    for (i, node) in enumerate(n)
        var_list = node_to_var[node]
        var_len = length(var_list)
        @variable(node, var_copy[1:var_len])

        for (j, var) in enumerate(var_copy)
            JuMP_var = var_list[j]
            var_to_var[JuMP_var] = var

            if is_binary(JuMP_var)
                JuMP.set_lower_bound(var, 0)
                JuMP.set_upper_bound(var, 1)
            elseif is_integer(JuMP_var)
                error("Contains integer variables")#set_integer(var)
            end

            if has_lower_bound(JuMP_var)
                set_lower_bound(var, lower_bound(JuMP_var))
            end
            if has_upper_bound(JuMP_var)
                set_upper_bound(var, upper_bound(JuMP_var))
            end

            if is_fixed(JuMP_var)
                fix(var, fix_value(JuMP_var); force = true)
            end
        end
    end

    all_cons = all_constraints(jm, include_variable_in_set_constraints = false)

    #co_types = unique(typeof.(constraint_object.(all_cons)))

    function add_new_var!(node, JuMP_var, var_to_var)
        new_var = @variable(node)
        var_to_var[JuMP_var] = new_var
        push!(node_to_var[node], var)
        var_to_node[var] = node

        if is_binary(JuMP_var)
            JuMP.set_lower_bound(var, 0)
            JuMP.set_upper_bound(var, 1)
        elseif is_integer(JuMP_var)
            error("Model has integer variables")
        end

        if has_lower_bound(JuMP_var)
            set_lower_bound(new_var, lower_bound(JuMP_var))
        end
        if has_upper_bound(JuMP_var)
            set_upper_bound(new_var, upper_bound(JuMP_var))
        end

        if is_fixed(JuMP_var)
            fix(new_var, fix_value(JuMP_var); force = true)
        end
    end

    function get_expr(con_obj, var_to_var)
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
        else
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
                add_new_var!(owning_node, new_var, var_to_var)
            end

            new_expr = sum(var_to_var[var] * con_obj.func.terms[var] for var in vars)
        end

        return new_expr, link, owning_node
    end

    for (i, con) in enumerate(all_cons)
        con_obj = constraint_object(con)
        if typeof(con_obj.set) == MOI.EqualTo{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var)
            if link
                @linkconstraint(graph, new_expr == con_obj.set.value)
            else
                @constraint(owning_node, new_expr == con_obj.set.value)
            end
        elseif typeof(con_obj.set) == MOI.LessThan{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var)
            if link
                @linkconstraint(graph, new_expr <= con_obj.set.upper)
            else
                @constraint(owning_node, new_expr <= con_obj.set.upper)
            end
        elseif typeof(con_obj.set) == MOI.GreaterThan{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var)
            if link
                @linkconstraint(graph, new_expr >= con_obj.set.lower)
            else
                @constraint(owning_node, new_expr >= con_obj.set.lower)
            end
        else
            error("constraint object does not match types")
        end
    end

    #TODO: Ensure that obj_func is aff not quad
    jm_obj_func = objective_function(jm)

    for (i, node) in enumerate(n)
        node_vars = intersect(node_to_var[node], jm_obj_func.terms.keys)
        new_obj_expr = sum(var_to_var[var] * jm_obj_func.terms[var] for var in node_vars)
        @objective(node, Min, new_obj_expr)
    end

    #new_obj_expr = sum(var_to_var[var] * jm_obj_func.terms[var] for var in keys(jm_obj_func.terms))

    #set_objective_function(graph, new_obj_expr)

    set_optimizer(graph, HiGHS.Optimizer)

    return graph, var_to_var, var_to_node, node_to_var
end
#=
function PSI_to_Plasmo_MIP_separation(model, horizon = 48)
    jm = model.internal.container.JuMPmodel
    av = all_variables(jm)

    names = name.(av)

    graph = OptiGraph()

    @optinode(graph, master)
    @optinode(graph, sub)


    var_numbers = zeros(2)

    node_to_var = Dict()
    var_to_node = Dict()
    var_to_var = Dict()

    node_to_var[master] = []
    node_to_var[sub] = []

    for (i, var) in enumerate(av)
        if is_binary(var)
            var_numbers[1] += 1
            push!(node_to_var[master], av[i])
            var_to_node[av[i]] = master
        else
            var_numbers[2] += 1
            push!(node_to_var[sub], av[i])
            var_to_node[av[i]] = master
        end
    end

    n = [master, sub]

    for (i, node) in enumerate(n)
        var_list = node_to_var[node]
        var_len = length(var_list)
        @variable(node, var_copy[1:var_len])

        for (j, var) in enumerate(var_copy)
            JuMP_var = var_list[j]
            var_to_var[JuMP_var] = var

            if is_binary(JuMP_var)
                JuMP.set_binary(var)
            elseif is_integer(JuMP_var)
                error("Contains integer variables")#set_integer(var)
            end

            if has_lower_bound(JuMP_var)
                set_lower_bound(var, lower_bound(JuMP_var))
            end
            if has_upper_bound(JuMP_var)
                set_upper_bound(var, upper_bound(JuMP_var))
            end

            if is_fixed(JuMP_var)
                fix(var, fix_value(JuMP_var); force = true)
            end
        end
    end

    all_cons = all_constraints(jm, include_variable_in_set_constraints = false)

    co_types = unique(typeof.(constraint_object.(all_cons)))

    function add_new_var!(node, JuMP_var, var_to_var)
        new_var = @variable(node)
        var_to_var[JuMP_var] = new_var
        push!(node_to_var[node], var)
        var_to_node[var] = node

        if is_binary(JuMP_var)
            JuMP.set_lower_bound(var, 0)
            JuMP.set_upper_bound(var, 1)
        elseif is_integer(JuMP_var)
            error("Model has integer variables")
        end

        if has_lower_bound(JuMP_var)
            set_lower_bound(new_var, lower_bound(JuMP_var))
        end
        if has_upper_bound(JuMP_var)
            set_upper_bound(new_var, upper_bound(JuMP_var))
        end

        if is_fixed(JuMP_var)
            fix(new_var, fix_value(JuMP_var); force = true)
        end
    end

    function get_expr(con_obj, var_to_var)
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
        else
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
                add_new_var!(owning_node, new_var, var_to_var)
            end

            new_expr = sum(var_to_var[var] * con_obj.func.terms[var] for var in vars)
        end

        return new_expr, link, owning_node
    end

    for (i, con) in enumerate(all_cons)
        con_obj = constraint_object(con)
        if typeof(con_obj.set) == MOI.EqualTo{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var)
            if link
                @linkconstraint(graph, new_expr == con_obj.set.value)
            else
                @constraint(owning_node, new_expr == con_obj.set.value)
            end
        elseif typeof(con_obj.set) == MOI.LessThan{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var)
            if link
                @linkconstraint(graph, new_expr <= con_obj.set.upper)
            else
                @constraint(owning_node, new_expr <= con_obj.set.upper)
            end
        elseif typeof(con_obj.set) == MOI.GreaterThan{Float64}
            new_expr, link, owning_node = get_expr(con_obj, var_to_var)
            if link
                @linkconstraint(graph, new_expr >= con_obj.set.lower)
            else
                @constraint(owning_node, new_expr >= con_obj.set.lower)
            end
        else
            error("constraint object does not match types")
        end
    end

    #TODO: Ensure that obj_func is aff not quad
    jm_obj_func = objective_function(jm)

    for (i, node) in enumerate(n)
        node_vars = intersect(node_to_var[node], jm_obj_func.terms.keys)
        new_obj_expr = sum(var_to_var[var] * jm_obj_func.terms[var] for var in node_vars)
        @objective(node, Min, new_obj_expr)
    end

    #new_obj_expr = sum(var_to_var[var] * jm_obj_func.terms[var] for var in keys(jm_obj_func.terms))

    #set_objective_function(graph, new_obj_expr)

    set_optimizer(graph, HiGHS.Optimizer)

    return graph, var_to_var, var_to_node, node_to_var
end

function add_node_copy(graph, expansion_set, var_to_var_rev)

    subs = getsubgraphs(graph)

    if length(subs) != length(expansion_set)
        error("Length of subgraphs does not match expansion set")
    end
    extra_obj = AffExpr[]

    for (j, sub) in enumerate(subs)
        expansions = expansion_set[j]
        node_to_node = Dict{OptiNode, OptiNode}()
        variable_map = Dict{VariableRef, VariableRef}()

        push!(extra_obj, AffExpr())

        for (k, node) in enumerate(expansions)
            node_copy = @optinode(sub)
            node_to_node[node] = node_copy
            vars = all_variables(node)
            variable_map = Dict{VariableRef, VariableRef}()
            sub_vars = all_variables(sub)

            for var in sub_vars
                variable_map[var] = var
            end

            copy_vars = @variable(node_copy, _copy_vars[1:length(vars)])

            for (l, var) in enumerate(vars)
                if JuMP.has_upper_bound(var)
                    set_upper_bound(copy_vars[l], upper_bound(var))
                end
                if JuMP.has_lower_bound(var)
                    set_lower_bound(copy_vars[l], lower_bound(var))
                end
                if JuMP.is_fixed(var)
                    fix(copy_vars[l], fix_value(var))
                end
                if JuMP.has_start_value(var)
                    set_start_value(copy_vars[l], start_value(var))
                end
                if JuMP.is_binary(var)
                    set_binary(copy_vars[l])
                end

                #@linkconstraint(graph, copy_vars[l] == var)
                variable_map[var] = copy_vars[l]
            end

            all_cons = all_constraints(node.model, include_variable_in_set_constraints = false)

            for (l, con) in enumerate(all_cons)
                con_obj = constraint_object(con)
                new_expr = sum(variable_map[var] * con_obj.func.terms[var] for var in keys(con_obj.func.terms))
                if typeof(con_obj.set) == MOI.EqualTo{Float64}
                    @constraint(node_copy, new_expr == con_obj.set.value)
                elseif typeof(con_obj.set) == MOI.LessThan{Float64}
                    @constraint(node_copy, new_expr <= con_obj.set.upper)
                elseif typeof(con_obj.set) == MOI.GreaterThan{Float64}
                    @constraint(node_copy, new_expr >= con_obj.set.lower)
                else
                    error("constraint object does not match types")
                end
            end

            obj_func = objective_function(node)
            if typeof(obj_func) == VariableRef
                obj_func = AffExpr(0, obj_func => 1)
            end

            node_copy_obj_func = AffExpr()

            for l in keys(obj_func.terms)

                #if occursin("Slack", name(var_to_var_rev[l]))
                #    node_copy_obj_func.terms[variable_map[l]] = obj_func.terms[l]
                #end

                if occursin("Slack", name(var_to_var_rev[l]))
                    obj_func.terms[l] = 0.5 * obj_func.terms[l]
                    node_copy_obj_func.terms[variable_map[l]] = obj_func.terms[l]
                    @linkconstraint(graph, l == variable_map[l])
                end
                #obj_func.terms[l] = 0.5 * obj_func.terms[l]
                #node_copy_obj_func.terms[variable_map[l]] = obj_func.terms[l]
                #@linkconstraint(graph, l == variable_map[l])
            end
            set_objective_function(node.model, obj_func)
            @objective(node_copy, Min, node_copy_obj_func)

            add_to_expression!(extra_obj[j], node_copy_obj_func)
        end

        Plasmo.set_graph_backend(sub)
        Plasmo.set_graph_backend(graph)

        incident_nodes = union(all_nodes(sub), expansions)

        in_edges = incident_edges(graph, incident_nodes)

        for (l, edge) in enumerate(in_edges)
            if all(x -> x in incident_nodes, edge.nodes)
                links = edge.linkrefs
                for (m, link) in enumerate(links)
                    con_obj = constraint_object(link)
                    new_expr = AffExpr()
                    for var in keys(con_obj.func.terms)
                        add_to_expression!(new_expr, variable_map[var], con_obj.func.terms[var])
                    end
                    #new_expr = sum(variable_map[var] * con_obj.func.terms[var] for var in keys(con_obj.func.terms)) # convert this to add_to_expression rather than summing
                    if typeof(con_obj.set) == MOI.EqualTo{Float64}
                        @linkconstraint(sub, new_expr == con_obj.set.value)
                    elseif typeof(con_obj.set) == MOI.LessThan{Float64}
                        @linkconstraint(sub, new_expr <= con_obj.set.upper)
                    elseif typeof(con_obj.set) == MOI.GreaterThan{Float64}
                        @linkconstraint(sub, new_expr >= con_obj.set.lower)
                    else
                        error("constraint object does not match types")
                    end
                end
            end
        end

        Plasmo.set_graph_backend(sub)
    end
    Plasmo.set_graph_backend(graph)

    return extra_obj
end
=#
