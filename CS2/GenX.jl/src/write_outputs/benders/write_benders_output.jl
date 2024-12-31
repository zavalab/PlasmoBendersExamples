function write_benders_output(LB_hist::Vector{Float64},UB_hist::Vector{Float64},cpu_time::Vector{Float64},feasibility_hist::Vector{Float64},outpath::AbstractString, setup::Dict,inputs::Dict,planning_problem::GenXModel)
	
	dfConv = DataFrame(Iter = 1:length(LB_hist),CPU_Time = cpu_time, LB = LB_hist, UB  = UB_hist, Gap = (UB_hist.-LB_hist)./LB_hist,Feasibility=feasibility_hist)

	if !has_values(planning_problem)
		optimize!(planning_problem)
	end
	
	write_planning_solution(outpath, inputs, setup, planning_problem)
	
	CSV.write(joinpath(outpath, "benders_convergence.csv"),dfConv)

	YAML.write_file(joinpath(outpath, "run_settings.yml"),setup)

end

