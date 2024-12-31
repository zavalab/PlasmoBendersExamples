# PlasmoBenders Examples

This repository includes scripts for the manuscript "Graph-Based Modeling and Decomposition of Hierarchical Optimization Problems". This paper presents a graph-theoretic framework for modeling hierarchical optimization problems. It uses the package [Plasmo.jl](https://github.com/plasmo-dev/Plasmo.jl/tree/main) (presented in [this paper](https://link.springer.com/article/10.1007/s12532-022-00223-3)) to construct optimization problems as a graph. The paper also introduces a graph-based Benders Decomposition (gBD) which is implemented in the package [PlasmoBenders.jl](https://github.com/plasmo-dev/PlasmoAlgorithms.jl/). 

## Repository Structure

This repository includes a set of tutorials (as presented in the manuscript) and three case studies. Details of the subdirectories are given below. 

* `Tutorials/` - Contains a set of basic tutorials for Plasmo.jl and PlasmoBenders.jl, including:
  * `storage_sizing.jl` - a hierarchical optimization example with a planning and an operational layer
  * `hierarchical_structures.jl` - an example of how to create hierarchical or nested graphs in Plasmo.jl
  * `partitioning_example.jl` - an example of how to partition graphs in Plasmo.jl
  * `aggregation_example.jl` - an example of how to aggregate graphs in Plasmo.jl
  * `plasmobenders_example.jl` - an example of how to use PlasmoBenders.jl for solving a hierarchical optimization problem
* `CS1` - A case study of a hierarchcial energy market, based on the work of [Atakan et al.](https://doi.org/10.1016/j.ejor.2021.12.042). This case study models the deterministic form of the problem presented by Atakan et al, where the problem is modeled using Plasmo.jl. It simulates 32 days of operation for a 118-bus system. This directory includes the following: 
  * `datasets/` - a subdirectory with data for the optimization problem
  * `data_interpolation.ipynb` - a Python notebook that interpolates the hourly data for the short term and hour ahead problems and converts it into 15-minute intervals. This is largely a reference file as the required data is already created by this file and stored in `datasets/`
  * `HP_process_time_data.jl` - a script for converting information from the datasets into the format needed for the optimization problem (this includes converting the loads from region-based to bus-based). This is largely a reference file as the required data is already created by this file and stored in `datasets/`
  * `HP_low_reserves/` - a directory containing the code for building the optimization problem. As described in the manuscript on hierarchical graph-based modeling, there are two solution approaches used in this problem - a receding horizon approach (also referred to as "serial" in the code) and a monolithic approach. This subdirectory includes the following: 
    * `gurobi_logs/` - The output of the Gurobi logs for each subproblem
    * `results/` - This is a directory where output CSVs are stored by the code. Due to its size, this data is not stored on this github repository but can be made available upon request
    * `32_day_data_analysis.jl` - a script for reading the CSVs in `results/` and outputting the results to CSVs for certain metrics like the number of generators committed in the day-ahead vs. short-term layers at a given time point
    * `get_objective_HA.jl` - a script for reading teh CSVs in `results/` and outputting the realized cost in the economic dispatch layer
    * `layer_construction.jl` - A set of functions for creating the initial subgraphs in Plasmo.jl and linking across some of the layers
    * `link_solutions.jl` - A set of functions for linking solutions across the subproblems for both receding horizon and monolithic problems
    * `run_32d_monolith.jl` - A script for running the monolithic problem
    * `run_32d_serial.jl` - A script for running the receding horizon problem
    * `test_up_down_times.jl` - A script for testing whether the up and down time constraints were properly formulated
* `CS2` - This case study is based on the [fourth example](https://github.com/GenXProject/GenX.jl/tree/main/example_systems/4_three_zones_w_policies_slack) from the package [GenX.jl](https://github.com/GenXProject/GenX.jl) (see [this paper](https://energy.mit.edu/wp-content/uploads/2017/10/Enhanced-Decision-Support-for-a-Changing-Electricity-Landscape.pdf)). At the time of writing, GenX.jl does not directly support Plasmo OptiGraphs, and the source code had to be updated to handle the plasmo.jl OptiGraph. In this case study, a capacity expansion problem, simulating the northeastern US over a period of 11 weeks, is solved using PlasmoBenders.jl.
  * `4_three_zones_w_policies_slack/` - This subdirectory contains the executable code for solving with Benders decomposition
    * `policies/`, `resources/`, `settings/`, `system/` - These folders contain data and setting information for the capacity expansion problem.
    * `run_plasmobenders.jl` - A script for building the capacity expansion problem in Plasmo.jl and solving wtih PlasmoBenders.jl
    * `benders_regularization.csv`, `benders_no_regularization.csv` - CSVs containing the results for Benders decomposition with and without regularization, including the upper and lower bound at each iteration and the time it took to solve each iteration.
  * `GenX.jl/` - This subdirectory contains the updated source code of GenX.jl to work with Plasmo.jl OptiGraphs
* `CS3` - This case study is a production cost model (PCM) constructed using [Sienna](https://github.com/NREL-Sienna) using the [PowerSimulations.jl](https://arxiv.org/abs/2404.03074v1) package. The specific problem instances are constructred using the code [here](https://github.com/NREL/LDES-Sizing-and-siting-model) which forms the PCM for a 5-bus and a 73-bus reliability test system (RTS). The resulting PCM is then converted into an OptiGraph in Plasmo.jl and solved using gBD.
  * `systems_data/` - set of possible system designs for the 5-bus and RTS problems. 
  * `scripts/` - A directory containing the scripts for constructing the PCMs.
    * `simulation_utils.jl` - A script with several supporting functions for building the PCM. These scripts are for working directly with PowerSystems.jl and PowerSimulations.jl and for loading in the systems in `systems_data/` and creating the PCM. 
    * `gephi_visuals.jl` - A script for exporting data for visualization in the software package [Gephi](https://gephi.org/) which was used for some of the visualizations of this case study. The script includes details on how the data is used in Gephi.
    * `PSI_to_plasmo.jl` - A script containing functions to convert the JuMP model that results from PowerSimulations.jl into a Plasmo.jl OptiGraph. Each time point of the PCM is treated as a node, and then time points are partitioned into different subgraphs for applying gBD. The directory also contains a script `PSI_to_Plasmo_high_res.jl` which is not required for this case study, but is used in `gephi_visuals.jl` which uses a higher resolution of the graph, where each bus and arc at each time point is represented by a Plasmo.jl OptiNode. 
    * `gBD_5bus_90_*.jl` - Scripts for solving the 5-bus system using PlasmoBenders.jl. Each system had a 90-day total time horizon. 
    * `gBD_RTS_15_*.jl` - Scripts for solving the RTS using PlasmoBenders.jl. Each system had a 15-day total time horizon.
    * `5bus_90d_Gurobi.jl` - Script for solving the 5-bus system without decomposition (for comparison with the PlasmoBenders scripts).
    * `RTS_15d_Gurobi.jl` - Script for solving the RTS without decomposition (for comparison with the PlasmoBenders scripts).
    * `*.csv` - CSVs containing the results of their corresponding case studies.
* `env/` - A Julia environment used for solving these problems. It can be accessed by activating and then instantiating the environment for replicating our results. If you are not familiar with how to activate and instantiate an environment, further details can be found [here](https://pkgdocs.julialang.org/v1/environments/).

## Versions

The above case studies were run with the following packages and versions. The specific environment we used can be accessed in the `env/` subfolder, which includes the `Manifest.toml` and `Project.toml` files. This environment can be accessed by activiting and then instantiating the environment. We used Julia version 1.9.2 for all case studies as well. 

Packages: 
 * CSV v0.10.15
 * DataFrames v1.6.1
 * DataStructures v0.18.20
 * DelimitedFiles v1.9.1
 * Distributions v0.25.113
 * Gurobi v1.3.0
 * HiGHS v1.12.1
 * HydroPowerSimulations v0.8.0
 * InfrastructureSystems v1.22.2
 * JuMP v1.23.4
 * Plasmo v0.6.4
 * PlasmoBenders v0.0.1
 * PowerSimulations v0.27.8
 * PowerSystems v3.3.0
 * Revise v3.6.3
 * StorageSystemsSimulations v0.9.0
 * Suppressor v0.2.8
 * TimeSeries v0.23.2