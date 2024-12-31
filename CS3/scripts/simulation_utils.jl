using PowerSystems
using PowerSimulations
using StorageSystemsSimulations
using InfrastructureSystems
using Dates
using Logging
using DataFrames
using CSV
const PSI = PowerSimulations
const IS = InfrastructureSystems
const PSY = PowerSystems
const SS = StorageSystemsSimulations
using Random, Distributions
using TimeSeries
using DataStructures

# Mapping command line arg to network formulation
get_day_ahead_timestamps(sim_year) = collect(DateTime("$(sim_year)-01-01T00:00:00"):Hour(1):DateTime("$(sim_year)-12-31T23:00:00"))

const NETWORK_MAP = Dict(
    "DCPPowerModel" => PSI.DCPPowerModel,
    "CopperPlatePowerModel" => PSI.CopperPlatePowerModel,
    "NFAPowerModel" => PSI.NFAPowerModel,
    "StandardPTDFModel" => PSI.PTDFPowerModel
)

# Mapping network formulation to the system balance constriant for retriving LMPs
const DAUL_MAP = Dict(
    "DCPPowerModel" => [NodalBalanceActiveConstraint],
    "CopperPlatePowerModel" => [CopperPlateBalanceConstraint],
    "NFAPowerModel" => [NodalBalanceActiveConstraint],
    "StandardPTDFModel" => [CopperPlateBalanceConstraint],
)

const DUAL_MAP_LINES = Dict(
    "DCPPowerModel" => [],
    "CopperPlatePowerModel" => [],
    "NFAPowerModel" => [],
    "StandardPTDFModel" => [NetworkFlowConstraint],
)

const BATTERY_MAP = Dict(
    "StorageDispatch" => SS.StorageDispatchWithReserves,
)

const THERMAL_MAP = Dict(
    "ThermalDispatchNoMin" => ThermalDispatchNoMin,
    "ThermalBasicUnitCommitment" => ThermalBasicUnitCommitment,
    "ThermalStandardUnitCommitment" => ThermalStandardUnitCommitment,
)

function get_template_uc(network_formulation, battery_form, add_ldes=true; add_slacks = true)
    template_uc = ProblemTemplate(
    NetworkModel(NETWORK_MAP[string(network_formulation)],
    duals=DAUL_MAP[string(network_formulation)],
    use_slacks=true))
    set_device_model!(template_uc, ThermalStandard, ThermalStandardUnitCommitment)
    set_device_model!(template_uc, ThermalMultiStart, ThermalBasicUnitCommitment)
    if add_ldes
        set_device_model!(template_uc, GenericBattery, BATTERY_MAP[string(battery_form)])
    end
    set_device_model!(template_uc, PSY.BatteryEMS, SS.StorageDispatchWithReserves)
    set_device_model!(template_uc, PowerLoad, StaticPowerLoad)
    set_device_model!(template_uc, RenewableDispatch, RenewableFullDispatch)
    set_device_model!(template_uc, MonitoredLine, StaticBranchBounds)
    set_device_model!(template_uc, DeviceModel(Line, StaticBranch; duals = DUAL_MAP_LINES[string(network_formulation)], use_slacks = add_slacks))
    set_device_model!(template_uc, DeviceModel(Transformer2W, StaticBranch; duals = DUAL_MAP_LINES[string(network_formulation)], use_slacks = add_slacks))
    set_device_model!(template_uc, DeviceModel(TapTransformer, StaticBranch; duals = DUAL_MAP_LINES[string(network_formulation)], use_slacks = add_slacks))
    set_service_model!(
        template_uc,
        ServiceModel(
            VariableReserve{ReserveUp},
            RangeReserve,
            use_slacks=true,
        )
    )

    return template_uc
end


function save_to_csv(cont, model_name, save_dir=".")
    for (name, df) in cont
        CSV.write(joinpath(save_dir, "$(name)_$(model_name).csv"), df)
    end
end

function export_results_csv(results, stage, path)
    variables = PSI.read_realized_variables(results)
    aux_variables = PSI.read_realized_aux_variables(results)
    parameters = PSI.read_realized_parameters(results)
    duals = PSI.read_realized_duals(results)
    expressions = PSI.read_realized_expressions(results)

    save_to_csv(variables, stage, path)
    save_to_csv(parameters, stage, path)
    save_to_csv(duals, stage, path)
    save_to_csv(aux_variables, stage, path)
    save_to_csv(expressions, stage, path)
    return
end

function deserialize_system(sys_path, run_on_eagle::Bool=false)
    if run_on_eagle
        sys = System(sys_path; time_series_directory="$(tempdir())")
    else
        sys = System(sys_path)
    end
    return sys
end

# This function fixes the order of the staticinjection objects in the adding forecasts functions
# for the 5 bus system; the other scripts in this repo for the 5 bus system move components around
# within the system, and this changes their order when adding the forecasts error and creates different
# forecasts between system configurations (even for the same devices); this ensures that the same
# devices have the same forecasts, even when they are moved around
function get_static_injectors(sys_da)
    if length(collect(get_components(RenewableDispatch, sys_da))) == 3
        rds = ["Wind", "Wind2", "SolarPV1"]
    else
        rds = ["Wind", "SolarPV1", "SolarPV2", "SolarPV3"]
    end

    comps = StaticInjection[]

    for i in ["node_b", "node_c", "node_d"]
        push!(comps, get_component(PowerLoad, sys_da, i))
    end
    for i in rds
        push!(comps, get_component(RenewableDispatch, sys_da, i))
    end
    for i in get_components(ThermalStandard, sys_da)
        push!(comps, i)
    end
    for i in get_components(GenericBattery, sys_da)
        push!(comps, i)
    end
    for i in get_components(BatteryEMS, sys_da)
        push!(comps, i)
    end
    return comps
end

function PSY.convert_component!(
    thtype::Type{<:PSY.ThermalGen},
    th::ThermalStandard,
    sys::System;
    kwargs...
)

    new_th = thtype(;
        name=th.name,
        available=th.available,
        status=th.status,
        bus=th.bus,
        active_power=th.active_power,
        reactive_power=th.reactive_power,
        rating=th.rating,
        prime_mover_type=th.prime_mover_type,
        fuel=th.fuel,
        active_power_limits=th.active_power_limits,
        reactive_power_limits=th.reactive_power_limits,
        ramp_limits=th.ramp_limits,
        time_limits=th.time_limits,
        operation_cost=th.operation_cost,
        base_power=th.base_power,
        power_trajectory=nothing,
        start_time_limits=nothing,
        start_types=1,
        services=Device[],
        must_run=true,
        time_at_status=th.time_at_status,
        dynamic_injector=th.dynamic_injector,
        ext=Dict{String,Any}(),
        time_series_container=InfrastructureSystems.TimeSeriesContainer(),
    )
    IS.assign_new_uuid!(new_th)
    add_component!(sys, new_th)
    copy_time_series!(new_th, th)
    for service in get_services(th)
        add_service!(new_th, service, sys)
    end
    remove_component!(sys, th)
    return
end

function convert_must_run_units!(sys)
    for d in get_components(x -> x.fuel == PSY.ThermalFuels.NUCLEAR, ThermalGen, sys)
        convert_component!(ThermalMultiStart, d, sys)
    end
end

function add_must_run_constraint!(model::PSI.DecisionModel)
    container = PSI.get_optimization_container(model)
    template = PSI.get_template(model)
    transmission_model = PSI.get_network_model(template)
    sys = PSI.get_system(model)
    if haskey(PSI.get_device_models(template), :ThermalMultiStart)
        device_model = PSI.get_device_models(template)[:ThermalMultiStart]
        devices = PSI.get_available_components(PSY.ThermalMultiStart, sys)
        PSI.add_constraints!(container, PSI.MustRunConstraint, devices, device_model, transmission_model)
    end
    return
end


function add_forecast_error(sys, device, time_series_name, distribution)
    data = SortedDict()
    ts = get_time_series(Deterministic, device, time_series_name)
    ts_data = get_time_series_values(SingleTimeSeries, device, time_series_name; ignore_scaling_factors=true)

    interval = Int(Dates.value(Hour(PSY.get_forecast_interval(sys))))
    horizon = Int(get_horizon(ts))
    for (i, init_time) in enumerate(PSY.get_forecast_initial_times(sys))
        _data = vcat(ts_data[(i-1)*interval+1 : interval*i], ts_data[(i*interval)+1 : (i-1)*interval+horizon] .+ rand(distribution, horizon - interval))
        data[init_time] = map(x -> x < 0.0 ? 0.0 : x,  _data)
    end
    device.ext[time_series_name] = data
    return
end


function add_forecast_error!(da_sys, error_mean)
    if length(collect(get_components(Bus, da_sys))) == 5
        components = vcat(collect(get_components(Service, da_sys)), get_static_injectors(da_sys))
    else
        components = vcat(collect(get_components(Service, da_sys)), collect(get_components(StaticInjection, da_sys)))
    end

    distribution = Normal(error_mean, 0.05)
    for d in components
        if isa(d, HybridSystem)
            d = d.renewable_unit
        end
        for ts_name in get_time_series_names(Deterministic, d)
            add_forecast_error(da_sys, d, ts_name, distribution)
        end
    end
    return
end

function add_single_time_series_forecast_error!(
    da_sys::PSY.System,
    horizon::Int,
    interval::Dates.Period,
    error_mean::Float64
)
    resolution = get_time_series_resolution(da_sys)
    transform_single_time_series!(da_sys, horizon, interval)
    add_forecast_error!(da_sys, error_mean)
    remove_time_series!(da_sys, Deterministic)
    remove_time_series!(da_sys, DeterministicSingleTimeSeries)
    if length(collect(get_components(Bus, da_sys))) == 5
        components = vcat(collect(get_components(Service, da_sys)), get_static_injectors(da_sys))
    else
        components = vcat(collect(get_components(Service, da_sys)), collect(get_components(StaticInjection, da_sys)))
    end
    for device in components
        for ts_name in get_time_series_names(SingleTimeSeries, device)
            if isa(device, HybridSystem)
                remove_time_series!(da_sys, SingleTimeSeries, device, ts_name)
                subcomponent = PSY.get_renewable_unit(device)
                sub_ts_name = "max_active_power"
                ts_data = PSY.Deterministic(
                    name = sub_ts_name,
                    data = subcomponent.ext[sub_ts_name],
                    resolution = resolution,
                    scaling_factor_multiplier = PSY.get_max_active_power,
                )

                if has_time_series(subcomponent,Deterministic)
                    remove_time_series!(da_sys, Deterministic, subcomponent, sub_ts_name)
                end
                add_time_series!(da_sys, subcomponent, ts_data)
                device.ext[ts_name] = nothing
                PSY.copy_subcomponent_time_series!(device, subcomponent)
            elseif typeof(device)  <: Service
                ts_data = PSY.Deterministic(
                    name = ts_name,
                    data = device.ext[ts_name],
                    resolution = resolution,
                    scaling_factor_multiplier = PSY.get_requirement,
                )
                add_time_series!(da_sys, device, ts_data)
                device.ext[ts_name] = nothing
            else
                ts_data = PSY.Deterministic(
                    name = ts_name,
                    data = device.ext[ts_name],
                    resolution = resolution,
                    scaling_factor_multiplier = PSY.get_max_active_power,
                )
                add_time_series!(da_sys, device, ts_data)
                device.ext[ts_name] = nothing
            end
        end
    end
    return da_sys
end


function _build_battery(::Type{T}, bus::PSY.Bus, name::String, energy_capacity, rating, efficiency) where {T<:PSY.Storage}
    device = T(;
        name=name,
        available=true,
        bus=bus,
        prime_mover=PSY.PrimeMovers.BA,
        initial_energy=energy_capacity / 2,
        state_of_charge_limits=(min=energy_capacity * 0.1, max=energy_capacity),
        rating=rating,
        active_power=rating,
        input_active_power_limits=(min=0.0, max=rating),
        output_active_power_limits=(min=0.0, max=rating),
        efficiency=(in=efficiency, out=1.0),
        reactive_power=0.0,
        reactive_power_limits=nothing,
        base_power=100.0,
        operation_cost=StorageManagementCost()
    )
    return device
end
