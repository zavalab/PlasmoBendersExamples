using Plasmo, PlasmoBenders, HiGHS

# Define a graph with three nodes and two edges
graph = OptiGraph()
@optinode(graph, nodes[1:3])

# Set local variable, constraint, and objective information
for node in nodes
    @variable(node, x, Bin)
    @variable(node, y >= 0)
    @constraint(node, x + y >= 1.3)
    @objective(node, Min, x + 2 * y)
end

# Add linking constriants
for i in 1:2
    @linkconstraint(graph, nodes[i][:x] + nodes[i + 1][:y] >= i)
end

# Create subgraphs with one node on each subgraph
part_vector = [1, 2, 3]
partition = Partition(graph, part_vector)
apply_partition!(graph, partition)

# Define a solver to use for the sub-problems
solver = optimizer_with_attributes(HiGHS.Optimizer, "output_flag" => false)

# Construct the BendersOptimizer object
benders_alg = BendersAlgorithm(
    graph, # Set the overall graph
    local_subgraphs(graph)[2]; # Set the root subgraph
    max_iters = 20, # Maximum iterations
    tol = 1e-7, # Solver tolerance
    parallelize_benders = true, # Whether to parallelize the second stage solutions
    strengthened = true, # Whether to use strengthened cuts
    regularize = false, # Whether to use a regularization scheme
    add_slacks = false, # Whether to add slack variables for recourse
    multicut = false, # Whether to use multi-cuts rather than aggregated
    solver = solver # Set the solver on the subgraphs
)

# Solve the problem using gBD
run_algorithm!(benders_alg)
