function write_simple_csv(filename::AbstractString, df::DataFrame)
    CSV.write(filename, df)
end

function write_simple_csv(filename::AbstractString, header::Vector, matrix)
    df = DataFrame(matrix, header)
    write_simple_csv(filename, df)
end

function prepare_timeseries_variables(EP::GenXModel, set::Set{Symbol}, scale::Float64 = 1.0)
    # function to extract data from DenseAxisArray
    data(var) = scale * value.(EP[var]).data

    return DataFrame(set .=> data.(set))
end

function write_timeseries_variables(EP, set::Set{Symbol}, filename::AbstractString)
    df = prepare_timeseries_variables(EP, set)
    write_simple_csv(filename, df)
end

@doc raw"""
    write_maintenance(path::AbstractString, inputs::Dict, EP::GenXModel)
"""
function write_maintenance(path::AbstractString, inputs::Dict, EP::GenXModel)
    downvars = maintenance_down_variables(inputs)
    write_timeseries_variables(EP, downvars, joinpath(path, "maint_down.csv"))
end
