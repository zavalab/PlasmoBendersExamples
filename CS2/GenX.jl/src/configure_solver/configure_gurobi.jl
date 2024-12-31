@doc raw"""
	configure_gurobi(solver_settings_path::String)

Reads user-specified solver settings from gurobi\_settings.yml in the directory specified by the string solver\_settings\_path.

Returns a `MathOptInterface.OptimizerWithAttributes` Gurobi optimizer instance to be used in the `GenX.generate_model()` method.


"""

function configure_gurobi(solver_settings_path::String, optimizer::Any)
    solver_settings = YAML.load(open(solver_settings_path))
    solver_settings = convert(Dict{String, Any}, solver_settings)
    default_settings = Dict("Crossover"=>0,"Method"=>2,"OutputFlag"=>1);
    attributes = merge(default_settings, solver_settings)
    attributes::Dict{String, Any}
    return optimizer_with_attributes(optimizer, attributes...)
end
