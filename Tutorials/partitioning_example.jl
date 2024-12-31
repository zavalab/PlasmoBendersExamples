# Define OptiGraph and nodes
g = OptiGraph()
@optinode(g, nodes[1:4])

# Add variables and objective to nodes
for node in nodes
    @variable(node, 0 <= x)
    @objective(node, Min, x)
end

# Add linking constraints between nodes
@linkconstraint(g, nodes[1][:x] + nodes[2][:x] >= 1)
@linkconstraint(g, nodes[2][:x] + nodes[3][:x] >= 1)
@linkconstraint(g, nodes[3][:x] + nodes[4][:x] >= 1)
@linkconstraint(g, nodes[4][:x] + nodes[1][:x] >= 1)

# Define node-membership vector and create its accompanying partition
node_membership_vector = [1, 1, 2, 2]
partition = Plasmo.Partition(g, node_membership_vector)

# Build an OptiGraph with subgraphs based on the partition
partitioned_g = Plasmo.assemble_optigraph(partition)

# Define a vector of OptiNodes and create its accompanying partition
optinode_vectors = [
    [nodes[1], nodes[2]],
    [nodes[3], nodes[4]]
]
partition = Plasmo.Partition(g, optinode_vectors)

# Apply the partition to the original graph
apply_partition!(g, partition)
