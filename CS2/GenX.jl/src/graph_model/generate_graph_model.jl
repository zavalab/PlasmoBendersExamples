function generate_graph_model(setup::Dict,inputs::Dict,PLANNING_OPTIMIZER::MOI.OptimizerWithAttributes,SUBPROB_OPTIMIZER::MOI.OptimizerWithAttributes)

    inputs_decomp = separate_inputs_subperiods(inputs);
    num_subproblems= length(inputs_decomp)

    graph = OptiGraph();
    @optinode(graph,planning_node)
    @optinode(graph,operation_nodes[1:num_subproblems])

    planning_variables = load_planning_node!(planning_node,setup,inputs)
    set_optimizer(planning_node, PLANNING_OPTIMIZER)

    linking_variables_maps = Dict{Int64,Dict{Plasmo.NodeVariableRef,Plasmo.NodeVariableRef}}()
    for i in 1:num_subproblems
        linking_variables_maps[i] = load_operation_node!(operation_nodes[i],setup,inputs_decomp[i],planning_variables)
        set_optimizer(operation_nodes[i], SUBPROB_OPTIMIZER)
    end

    make_linking_constraints!(graph,linking_variables_maps)

    return graph,linking_variables_maps
end

function load_planning_node!(EP::JuMP.Model,setup::Dict,inputs::Dict)
    num_subperiods = inputs["REP_PERIOD"];
    @variable(EP, vZERO==0)
    @expression(EP,eObj,AffExpr(0.0))
    planning_model!(EP,setup,inputs)
    @variable(EP,vTHETA[1:num_subperiods]>=0)
    @objective(EP, Min, setup["ObjScale"]*(EP[:eObj]+sum(EP[:vTHETA])))
    planning_vars = setdiff(all_variables(EP),[EP[:vZERO];EP[:vTHETA]]);
    return planning_vars

end

function load_planning_node!(EP::Plasmo.OptiNode,setup::Dict,inputs::Dict)
    num_subperiods = inputs["REP_PERIOD"];
    @variable(EP, vZERO==0)
    @expression(EP,eObj,GenericAffExpr{Float64, Plasmo.NodeVariableRef}(0.0))
    planning_model!(EP,setup,inputs)
    @variable(EP,vTHETA[1:num_subperiods]>=0)
    @objective(EP, Min, setup["ObjScale"]*(EP[:eObj]+sum(EP[:vTHETA])))
    planning_vars = setdiff(all_variables(EP),[EP[:vZERO];EP[:vTHETA]]);
    return planning_vars

end

function load_operation_node!(EP::JuMP.Model,setup::Dict,inputs::Dict,planning_variables::Vector{VariableRef})

    @variable(EP, vZERO==0)
    @expression(EP,eObj,AffExpr(0.0))
    operation_model!(EP,setup,inputs)
    @objective(EP, Min, setup["ObjScale"]*EP[:eObj])

    all_vars = all_variables(EP);
    var_strings = name.(all_vars)
    clean_var_strings = [replace(str,r"operation_nodes\[" * string(inputs["SubPeriod"]) * r"\]" * r"\[:(.*)\]"=>s"\1") for str in var_strings]
    planning_var_strings = name.(planning_variables)
    clean_planning_var_strings = [replace(str,r"planning_node\[:(.*)\]"=>s"\1") for str in planning_var_strings]

    common_subproblem_indices = [findfirst(clean_var_strings.==s) for s in intersect(clean_var_strings,clean_planning_var_strings)]
    linking_variables_map = Dict(all_vars[i]=>planning_variables[findfirst(clean_planning_var_strings.==clean_var_strings[i])] for i in common_subproblem_indices)

    return linking_variables_map
end

function load_operation_node!(EP::Plasmo.OptiNode,setup::Dict,inputs::Dict,planning_variables::Vector{Plasmo.NodeVariableRef})

    @variable(EP, vZERO==0)
    @expression(EP,eObj,GenericAffExpr{Float64, Plasmo.NodeVariableRef}(0.0))
    operation_model!(EP,setup,inputs)
    @objective(EP, Min, setup["ObjScale"]*EP[:eObj])

    all_vars = all_variables(EP);
    var_strings = name.(all_vars)
    clean_var_strings = [replace(str,r"operation_nodes\[" * string(inputs["SubPeriod"]) * r"\]" * r"\[:(.*)\]"=>s"\1") for str in var_strings]
    planning_var_strings = name.(planning_variables)
    clean_planning_var_strings = [replace(str,r"planning_node\[:(.*)\]"=>s"\1") for str in planning_var_strings]

    common_subproblem_indices = [findfirst(clean_var_strings.==s) for s in intersect(clean_var_strings,clean_planning_var_strings)]
    linking_variables_map = Dict(all_vars[i]=>planning_variables[findfirst(clean_planning_var_strings.==clean_var_strings[i])] for i in common_subproblem_indices)

    return linking_variables_map
end


function make_linking_constraints!(graph::OptiGraph,linking_variables_maps::Dict{Int64,Dict{Plasmo.NodeVariableRef,Plasmo.NodeVariableRef}})
    for i in keys(linking_variables_maps)
        for k in keys(linking_variables_maps[i])
            @linkconstraint(graph, k == linking_variables_maps[i][k])
        end
    end
end
