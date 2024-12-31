
function write_planning_solution(path::AbstractString, inputs::Dict, setup::Dict, planning_problem::GenXModel)

    existingcap = value.(planning_problem[:eExistingCap])
	totcap = value.(planning_problem[:eTotalCap]);

    existingcapenergy = zeros(length(inputs["RESOURCE_NAMES"]))
	totcapenergy = zeros(length(inputs["RESOURCE_NAMES"]))

    existingcapcharge = zeros(length(inputs["RESOURCE_NAMES"]))
	totcapcharge = zeros(length(inputs["RESOURCE_NAMES"]))
    
    for i in inputs["STOR_ASYMMETRIC"]
        existingcapcharge[i] = value(planning_problem[:eExistingCapCharge][i]);
		totcapcharge[i] = value(planning_problem[:eTotalCapCharge][i])
    end
    
    for i in inputs["STOR_ALL"]
        existingcapenergy[i] = value(planning_problem[:eExistingCapEnergy][i]);
		totcapenergy[i] = value(planning_problem[:eTotalCapEnergy][i])
    end

	scale_factor = setup["ParameterScale"] == 1 ? ModelScalingFactor : 1

	existingcap = existingcap.*scale_factor;
	existingcapcharge = existingcapcharge.*scale_factor;
	existingcapenergy = existingcapenergy.*scale_factor;
	totcap = totcap.*scale_factor;
	totcapcharge = totcapcharge.*scale_factor;
	totcapenergy = totcapenergy.*scale_factor;

    auxnames = ["Resource";"ExistingCap_MW";"EndCap_MW"]
    dfCap = DataFrame([inputs["RESOURCE_NAMES"] existingcap totcap],auxnames)
    CSV.write(joinpath(path, "master_sol_capacity.csv"), dfCap)

	if !isempty(inputs["STOR_ASYMMETRIC"])
		auxnames_charge = ["Resource";"ExistingCapCharge_MW";"EndCapCharge_MW"];
    	dfCapCharge = DataFrame([inputs["RESOURCE_NAMES"] existingcapcharge totcapcharge],auxnames_charge)
    	CSV.write(joinpath(path, "master_sol_capacity_charge.csv"), dfCapCharge)
	end

	if !isempty(inputs["STOR_ALL"])
		auxnames_energy = ["Resource";"ExistingCapEnergy_MWh";"EndCapEnergy_MWh"]
    	dfCapEnergy = DataFrame([inputs["RESOURCE_NAMES"] existingcapenergy totcapenergy],auxnames_energy)
		CSV.write(joinpath(path, "master_sol_capacity_energy.csv"), dfCapEnergy)
	end

    existingtranscap = inputs["pTrans_Max"]*scale_factor;
	tottranscap = value.(planning_problem[:eAvail_Trans_Cap])*scale_factor;

	auxnames = ["Line";"ExistingTransCap_MW";"EndTransCap_MW"]
	dfTransCap = DataFrame([ ["L$i" for i in 1: inputs["L"]] existingtranscap tottranscap],auxnames)

    CSV.write(joinpath(path, "master_sol_trans_capacity.csv"), dfTransCap)

	### Writing costs
	cFix = value(planning_problem[:eTotalCFix]) + (!isempty(inputs["STOR_ALL"]) ? value(planning_problem[:eTotalCFixEnergy]) : 0.0) + (!isempty(inputs["STOR_ASYMMETRIC"]) ? value(planning_problem[:eTotalCFixCharge]) : 0.0)

	cNetworkExp = value.(planning_problem[:eTotalCNetworkExp])
	
 	# Conversion from Million$ to $
	cFix *= scale_factor^2;
	cNetworkExp *= scale_factor^2;

	dfCost = DataFrame([cFix cNetworkExp],["FixCost";"NetworkExpCost"])
	
	CSV.write(joinpath(path, "master_sol_costs.csv"), dfCost)

end

