# Load in packages
using Plasmo, HiGHS

# Set Problem data
T = 20
gamma = fill(5, T); beta = fill(20, T); alpha = 10; zeta = 2
gamma[8:10] .= 20; gamma[16:20] .= 50
d_sell = 50; d_save = 20; d_buy = 15; y_bar = 10
            
# Instantiate graph and nodes
graph = OptiGraph()
@optinode(graph, planning_node)
@optinode(graph, operation_nodes[1:T])
            
# Define planning node data which includes storage size
@variable(planning_node, storage_size >= 0)
@objective(planning_node, Min, storage_size * alpha)
            
# Loop through operations nodes and set variables, constraint, and objective
for (j, node) in enumerate(operation_nodes)
    @variable(node, 0 <= y_stored)
    @variable(node, 0 <= y_sell <= d_sell)
    @variable(node, -d_save <= y_save <= d_save)
    @variable(node, 0 <= x_buy <= d_buy)
            
    @constraint(node, y_save + y_sell - zeta * x_buy == 0)
    @objective(node, Min, x_buy * beta[j] - y_sell * gamma[j])
end

# Set initial storage level
@constraint(operation_nodes[1], operation_nodes[1][:y_stored] == y_bar)
            
# Set mass balance on storage unit
@linkconstraint(graph, [i = 1:(T - 1)], operation_nodes[i + 1][:y_stored] -
                        operation_nodes[i][:y_stored] == operation_nodes[i + 1][:y_save])
                        
# Link planning decision to operations decisiosn
@linkconstraint(graph, [i = 1:T], operation_nodes[i][:y_stored] <= planning_node[:storage_size])
            
# Set graph objective
set_to_node_objectives(graph)
            
# Optimizer the graph
set_optimizer(graph, HiGHS.Optimizer)
optimize!(graph)
