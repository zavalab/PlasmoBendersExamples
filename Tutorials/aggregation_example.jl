# Define a function for building a graph with a specific partition
function build_partitioned_graph()
    g = OptiGraph()
    @optinode(g, nodes[1:4])

    for node in nodes
        @variable(node, 0 <= x)
        @objective(node, Min, x)
    end
    @linkconstraint(g, nodes[1][:x] + nodes[2][:x] >= 1)
    @linkconstraint(g, nodes[2][:x] + nodes[3][:x] >= 1)
    @linkconstraint(g, nodes[3][:x] + nodes[4][:x] >= 1)
    @linkconstraint(g, nodes[4][:x] + nodes[1][:x] >= 1)

    node_membership_vector = [1, 1, 2, 2]
    partition = Plasmo.Partition(g, node_membership_vector)

    apply_partition!(g, partition)
    return g
end

# Define an overall graph and a set of partitioned subgraphs
g = OptiGraph()
g1 = build_partitioned_graph()
g2 = build_partitioned_graph()

# Add the partitioned subgraphs to the overall graph g
add_subgraph!(g, g1)
add_subgraph!(g, g2)

# Add extra nodes to each graph
@optinode(g, n_g)
@optinode(g1, n_g1)
@optinode(g2, n_g2)

# Aggregate the overall graph into a single node
agg_node, ref_map = aggregate(g)
agg_graph = source_graph(agg_node)

# Alternatively aggregate the subgraphs at different depth levels
agg_graph_layer0, ref_map_layer0 = aggregate_to_depth(g, 0)
agg_graph_layer1, ref_map_layer1 = aggregate_to_depth(g, 1)
