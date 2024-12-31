using Plasmo

# Define function for creating subgraphs with 3 nodes
function build_graph()
    # Define OptiGraph
    g = OptiGraph()
            
    # Add nodes
    @optinode(g, nodes[1:3])
            
    # Set variables on nodes
    @variable(nodes[1], x >= 0)
    @variable(nodes[2], x >= 0)
    @variable(nodes[3], x >= 0)
            
    # Set objective on nodes
    @objective(nodes[1], Min, 1 * nodes[1][:x])
    @objective(nodes[2], Min, 2 * nodes[2][:x])
    @objective(nodes[3], Min, 3 * nodes[3][:x])
            
    # Define link constraints between nodes
    @linkconstraint(g, nodes[1][:x] + nodes[2][:x] >= 1)
    @linkconstraint(g, nodes[1][:x] + nodes[3][:x] >= 1)
    @linkconstraint(g, nodes[2][:x] + nodes[3][:x] >= 1)

    set_to_node_objectives(g)
            
    # Return graph
    return g
end 
            
# Define high-level and mid-level OptiGraphs
g = OptiGraph(); g1 = OptiGraph(); g2 = OptiGraph();
            
# Define lower-level OptiGraphs that contain nodes and edges
g11 = build_graph(); g12 = build_graph(); 
g21 = build_graph(); g22 = build_graph();
g23 = build_graph(); g24 = build_graph(); 
            
# Add lower-level subgraphs to mid-level
add_subgraph!(g1, g11); add_subgraph!(g1, g12);
add_subgraph!(g2, g21); add_subgraph!(g1, g22);
add_subgraph!(g2, g23); add_subgraph!(g1, g24);
            
# Define link constraints between lower-level subgraphs;
# these constraints are "owned" by the mid-level graphs
@linkconstraint(g1, g11[:nodes][1][:x] + g12[:nodes][1][:x] >= 1)
@linkconstraint(g1, g11[:nodes][3][:x] + g12[:nodes][2][:x] >= 1)
            
@linkconstraint(g2, g21[:nodes][1][:x] + g22[:nodes][1][:x] >= 1)
@linkconstraint(g2, g21[:nodes][3][:x] + g22[:nodes][1][:x] >= 1)
            
@linkconstraint(g2, g22[:nodes][1][:x] + g23[:nodes][1][:x] >= 1)
@linkconstraint(g2, g22[:nodes][3][:x] + g23[:nodes][1][:x] >= 1)
            
@linkconstraint(g2, g23[:nodes][1][:x] + g24[:nodes][1][:x] >= 1)
@linkconstraint(g2, g23[:nodes][3][:x] + g24[:nodes][1][:x] >= 1)
            
# Add mid-level subgraphs to high-level OptiGraph
add_subgraph!(g, g1); add_subgraph!(g, g2);

@linkconstraint(g, g11[:nodes][1][:x] + g21[:nodes][1][:x] >= 1)
@linkconstraint(g, g11[:nodes][1][:x] + g22[:nodes][1][:x] >= 1)
@linkconstraint(g, g12[:nodes][1][:x] + g23[:nodes][1][:x] >= 1)
@linkconstraint(g, g12[:nodes][1][:x] + g24[:nodes][1][:x] >= 1)

set_to_node_objectives(g)
set_to_node_objectives(g1)
set_to_node_objectives(g2)
