using Revise
using PowerSystems
using PowerSimulations
using Dates
using Logging
using DataFrames
using CSV
using StorageSystemsSimulations
using InfrastructureSystems
logger = configure_logging(console_level=Logging.Info)
const PSI = PowerSimulations
const PSY = PowerSystems
const SS = StorageSystemsSimulations
const IS = InfrastructureSystems
using TimeSeries
using JuMP
using HiGHS
using Gurobi
using Random

# Define seed (for adding noise to forecast horizons)
Random.seed!(10)

include((@__DIR__)*"/simulation_utils.jl")

# Set system location
sys_name = (@__DIR__)*"/../systems_data/5bus_system_Wind_caseB.json"

# Set problem attributes
interval = 24 # simulation step interval in Hours
num_periods = 1
horizon = 24 * 90 # Simulation horizon in hours
steps = 1 # Number of steps in simulation
battery = true # Include LDES
form = "StorageDispatch" # Battery model to use
network_formulation = "StandardPTDFModel" # Network model to use
output_dir = (@__DIR__)*"/5bus_sims/Gurobi_90d" # Where to put the output directory

# Define solver to use
solver = optimizer_with_attributes(Gurobi.Optimizer, "MIPGap" => 1e-3)

# Build the output directory if it is not there
if !ispath(output_dir)
    mkpath(output_dir)
end

# Define the problem template
# used by PSI to define the optimization problem
template_uc = get_template_uc(network_formulation, "StorageDispatch")

# Load in the PSY system
sys_UC = System(sys_name)

# Set the units in PSY
set_units_base_system!(sys_UC, PSY.UnitSystem.SYSTEM_BASE)

# Add forecast errors for simulations
Random.seed!(10)
add_single_time_series_forecast_error!(sys_UC, horizon, Hour(interval), 0.05)

# Define models and simulations
models = SimulationModels(
    decision_models=[
        DecisionModel(template_uc,
            sys_UC,
            name="UC",
            optimizer=solver,
            initialize_model=false,
            optimizer_solve_log_print=true,
            direct_mode_optimizer=true,
            check_numerical_bounds=false,
            warm_start=true,
        ),
    ],
)

# Define the simulation sequence (for PSI)
sequence = SimulationSequence(
    models=models,
    ini_cond_chronology=InterProblemChronology(),
)

# Build the simulation in PSI
# This will build the JuMP model
sim = Simulation(
    name="test",
    steps=steps,
    models=models,
    sequence=sequence,
    simulation_folder=output_dir,
)
build!(sim, console_level=Logging.Info, file_level=Logging.Info)

# Get the model object from the simulation object
model = get_simulation_model(sim, :UC)

# Get the JuMP model from the PSI Model
jm = model.internal.container.JuMPmodel

# Solve the model object
t = @elapsed optimize!(jm)

println("Time to run the model was $(t / 60) min")
println("The objective value was ", objective_value(jm))
println("Termination status was ", termination_status(jm))
println("The number of variables was ", length(all_variables(jm)))
