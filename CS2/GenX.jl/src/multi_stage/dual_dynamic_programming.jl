@doc raw"""
    configure_ddp_dicts(setup::Dict, inputs::Dict)

This function instantiates Dictionary objects containing the names of linking expressions, constraints, and variables used in multi-stage modeling.

inputs:

* setup - Dictionary object containing GenX settings and key parameters.
* inputs – Dictionary of inputs for each model period, generated by the load\_inputs() method.

returns:

* start\_cap\_d – Dictionary which contains linking expression names as keys and linking constraint names as values, used for setting the end capacity in stage $p$ to the starting capacity in stage $p+1$.
* cap\_track\_d – Dictionary which contains linking variable names as keys and linking constraint names as values, used for enforcing endogenous retirements.
"""
function configure_ddp_dicts(setup::Dict, inputs::Dict)

    # start_cap_d dictionary contains key-value pairs of available capacity investment expressions
    # as keys and their corresponding linking constraints as values
    start_cap_d = Dict([(Symbol("eTotalCap"), Symbol("cExistingCap"))])

    if !isempty(inputs["STOR_ALL"])
        start_cap_d[Symbol("eTotalCapEnergy")] = Symbol("cExistingCapEnergy")
    end

    if !isempty(inputs["STOR_ASYMMETRIC"])
        start_cap_d[Symbol("eTotalCapCharge")] = Symbol("cExistingCapCharge")
    end

    if setup["NetworkExpansion"] == 1 && inputs["Z"] > 1
        start_cap_d[Symbol("eAvail_Trans_Cap")] = Symbol("cExistingTransCap")
    end

    if !isempty(inputs["VRE_STOR"])
        if !isempty(inputs["VS_DC"])
            start_cap_d[Symbol("eTotalCap_DC")] = Symbol("cExistingCapDC")
        end

        if !isempty(inputs["VS_SOLAR"])
            start_cap_d[Symbol("eTotalCap_SOLAR")] = Symbol("cExistingCapSolar")
        end

        if !isempty(inputs["VS_WIND"])
            start_cap_d[Symbol("eTotalCap_WIND")] = Symbol("cExistingCapWind")
        end

        if !isempty(inputs["VS_STOR"])
            start_cap_d[Symbol("eTotalCap_STOR")] = Symbol("cExistingCapEnergy_VS")
        end

        if !isempty(inputs["VS_ASYM_DC_DISCHARGE"])
            start_cap_d[Symbol("eTotalCapDischarge_DC")] = Symbol("cExistingCapDischargeDC")
        end

        if !isempty(inputs["VS_ASYM_DC_CHARGE"])
            start_cap_d[Symbol("eTotalCapCharge_DC")] = Symbol("cExistingCapChargeDC")
        end

        if !isempty(inputs["VS_ASYM_AC_DISCHARGE"])
            start_cap_d[Symbol("eTotalCapDischarge_AC")] = Symbol("cExistingCapDischargeAC")
        end

        if !isempty(inputs["VS_ASYM_AC_CHARGE"])
            start_cap_d[Symbol("eTotalCapCharge_AC")] = Symbol("cExistingCapChargeAC")
        end
    end

    # This dictionary contains the endogenous retirement constraint name as a key,
    # and a tuple consisting of the associated tracking array constraint and variable as the value
    cap_track_d = Dict([(Symbol("vCAPTRACK"), Symbol("cCapTrack"))])

    if !isempty(inputs["STOR_ALL"])
        cap_track_d[Symbol("vCAPTRACKENERGY")] = Symbol("cCapTrackEnergy")
    end

    if !isempty(inputs["STOR_ASYMMETRIC"])
        cap_track_d[Symbol("vCAPTRACKCHARGE")] = Symbol("cCapTrackCharge")
    end

    if !isempty(inputs["VRE_STOR"])
        if !isempty(inputs["VS_DC"])
            cap_track_d[Symbol("vCAPTRACKDC")] = Symbol("cCapTrackDC")
        end

        if !isempty(inputs["VS_SOLAR"])
            cap_track_d[Symbol("vCAPTRACKSOLAR")] = Symbol("cCapTrackSolar")
        end

        if !isempty(inputs["VS_WIND"])
            cap_track_d[Symbol("vCAPTRACKWIND")] = Symbol("cCapTrackWind")
        end

        if !isempty(inputs["VS_STOR"])
            cap_track_d[Symbol("vCAPTRACKENERGY_VS")] = Symbol("cCapTrackEnergy_VS")
        end

        if !isempty(inputs["VS_ASYM_DC_DISCHARGE"])
            cap_track_d[Symbol("vCAPTRACKDISCHARGEDC")] = Symbol("cCapTrackDischargeDC")
        end

        if !isempty(inputs["VS_ASYM_DC_CHARGE"])
            cap_track_d[Symbol("vCAPTRACKCHARGEDC")] = Symbol("cCapTrackChargeDC")
        end

        if !isempty(inputs["VS_ASYM_AC_DISCHARGE"])
            cap_track_d[Symbol("vCAPTRACKDISCHARGEAC")] = Symbol("cCapTrackDischargeAC")
        end

        if !isempty(inputs["VS_ASYM_AC_CHARGE"])
            cap_track_d[Symbol("vCAPTRACKCHARGEAC")] = Symbol("cCapTrackChargeAC")
        end
    end

    return start_cap_d, cap_track_d
end

@doc raw"""
	run_ddp(models_d::Dict, setup::Dict, inputs_d::Dict)

This function run the dual dynamic programming (DDP) algorithm, as described in [Pereira and Pinto (1991)](https://doi.org/10.1007/BF01582895), and more recently, [Lara et al. (2018)](https://doi.org/10.1016/j.ejor.2018.05.039). Note that if the algorithm does not converge within 10,000 (currently hardcoded) iterations, this function will return models with sub-optimal solutions. However, results will still be printed as if the model is finished solving. This sub-optimal termination is noted in the output with the 'Exiting Without Covergence!' message.

inputs:

  * models\_d – Dictionary which contains a JuMP model for each model period.
  * setup - Dictionary object containing GenX settings and key parameters.
  * inputs\_d – Dictionary of inputs for each model stage, generated by the load\_inputs() method.

returns:

  * models\_d – Dictionary which contains a JuMP model for each model stage, modified by this method.
  * stats\_d – Dictionary which contains the run time, upper bound, and lower bound of each DDP iteration.
  * inputs\_d – Dictionary of inputs for each model stage, generated by the load\_inputs() method, modified by this method.
"""
function run_ddp(outpath::AbstractString, models_d::Dict, setup::Dict, inputs_d::Dict)
    settings_d = setup["MultiStageSettingsDict"]
    num_stages = settings_d["NumStages"]  # Total number of investment planning stages
    EPSILON = settings_d["ConvergenceTolerance"] # Tolerance
    myopic = settings_d["Myopic"] == 1 # 1 if myopic (only one forward pass), 0 if full DDP

    start_cap_d, cap_track_d = configure_ddp_dicts(setup, inputs_d[1])

    ic = 0 # Iteration Counter

    results_d = Dict() # Dictionary to store the results to return
    times_a = [] # Array to store the total time of each iteration
    upper_bounds_a = [] # Array to store the upper bound of each iteration
    lower_bounds_a = [] # Array to store the lower bound of each iteration
    stats_d = Dict() # Dictionary to store the statistics (total time, upper bound, and lower bound for each iteration)
    stats_d["TIMES"] = times_a
    stats_d["UPPER_BOUNDS"] = upper_bounds_a
    stats_d["LOWER_BOUNDS"] = lower_bounds_a

    # Step a.i) Initialize cost-to-go function for t = 1:num_stages
    for t in 1:num_stages
        settings_d["CurStage"] = t
        models_d[t] = initialize_cost_to_go(settings_d, models_d[t], inputs_d[t])
    end

    # Step a.ii) Set objective upper bound
    global z_upper = Inf

    # Step b.i) Solve the approximate first-stage problem
    println("***********")
    println("Solving First Stage Problem")
    println("***********")

    t = 1 # Stage = 1
    solve_time_d = Dict()
    ddp_prev_time = time() # Begin tracking time of each iteration
    models_d[t], solve_time_d[t] = solve_model(models_d[t], setup)
    inputs_d[t]["solve_time"] = solve_time_d[t]

    # Step c.i) Initialize the lower bound, equal to the objective function value for the first period in the first iteration
    global z_lower = objective_value(models_d[t])

    # Step c.ii) If the relative difference between upper and lower bounds are small, break loop
    while ((z_upper - z_lower) / z_lower > EPSILON)
        ic = ic + 1 # Increase iteration counter by 1

        if (ic > 10000)
            println("***********")
            println("Exiting Without Covergence!")
            println(string("Upper Bound = ", z_upper))
            println(string("Lower Bound = ", z_lower))
            println("***********")

            return models_d, stats_d, inputs_d
        end

        println("***********")
        println(string("Iteration Number: ", ic))
        println(string("Upper Bound = ", z_upper))
        println(string("Lower Bound = ", z_lower))
        println("***********")

        # Step d) Forward pass for t = 1:num_stages
        ## For first iteration we dont need to solve forward pass for first stage (we did that already above),
        ## but we need to update forward pass solution for the first stage for subsequent iterations
        if ic > 1
            t = 1 #  update forward pass solution for the first stage
            models_d[t], solve_time_d[t] = solve_model(models_d[t], setup)
            inputs_d[t]["solve_time"] = solve_time_d[t]
        end
        ## Forward pass for t=2:num_stages
        for t in 2:num_stages
            println("***********")
            println(string("Forward Pass t = ", t))
            println("***********")

            # Step d.i) Fix initial investments for model at time t given optimal solution for time t-1
            models_d[t] = fix_initial_investments(models_d[t - 1],
                models_d[t],
                start_cap_d,
                inputs_d[t])

            # Step d.ii) Fix capacity tracking variables for endogenous retirements
            models_d[t] = fix_capacity_tracking(models_d[t - 1],
                models_d[t],
                cap_track_d,
                t)

            # Step d.iii) Solve the model at time t
            models_d[t], solve_time_d[t] = solve_model(models_d[t], setup)
            inputs_d[t]["solve_time"] = solve_time_d[t]
        end

        ### For the myopic solution, algorithm should terminate here after the first forward pass calculation and then move to Outputs writing.
        if myopic
            println("***********")
            println("Exiting After First Forward Pass! (Myopic)")
            println(string("Upper Bound = ", z_upper))
            println(string("Lower Bound = ", z_lower))
            println("***********")
            return models_d, stats_d, inputs_d
        end
        ###

        # Step e) Calculate the new upper bound
        z_upper_temp = 0
        for t in 1:num_stages
            z_upper_temp = z_upper_temp +
                           (objective_value(models_d[t]) - value(models_d[t][:vALPHA]))
        end

        # If the upper bound decreased, set it as the new upper bound
        if z_upper_temp < z_upper
            z_upper = z_upper_temp
        end

        append!(upper_bounds_a, z_upper) # Store current iteration upper bound
        update_multi_stage_stats_file(outpath, ic, z_upper, z_lower, NaN, new_row = true)

        # Step f) Backward pass for t = num_stages:2
        for t in num_stages:-1:2
            println("***********")
            println(string("Backward Pass t = ", t))
            println("***********")

            # Step f.i) Add a cut to the previous time step using information from the current time step
            models_d[t - 1] = add_cut(models_d[t - 1],
                models_d[t],
                start_cap_d,
                cap_track_d)

            # Step f.ii) Solve the model with the additional cut at time t-1
            models_d[t - 1], solve_time_d[t - 1] = solve_model(models_d[t - 1], setup)
            inputs_d[t - 1]["solve_time"] = solve_time_d[t - 1]
        end

        # Step g) Recalculate lower bound and go back to c)
        z_lower = objective_value(models_d[1])
        append!(lower_bounds_a, z_lower) # Store current iteration lower bound
        update_multi_stage_stats_file(outpath, ic, z_upper, z_lower, NaN)

        # Step h) Store the total time of the current iteration (in seconds)
        ddp_iteration_time = time() - ddp_prev_time
        append!(times_a, ddp_iteration_time)
        update_multi_stage_stats_file(outpath, ic, z_upper, z_lower, ddp_iteration_time)

        ddp_prev_time = time()
    end

    println("***********")
    println("Successful Convergence!")
    println(string("Upper Bound = ", z_upper))
    println(string("Lower Bound = ", z_lower))
    println("***********")

    ### STEP I) One final forward pass to guarantee convergence
    # Forward pass for t = 1:num_stages
    t = 1 #  update forward pass solution for the first stage
    models_d[t], solve_time_d[t] = solve_model(models_d[t], setup)
    inputs_d[t]["solve_time"] = solve_time_d[t]
    ## Forward pass for t=2:num_stages
    for t in 2:num_stages
        println("***********")
        println(string("Final Forward Pass t = ", t))
        println("***********")

        # Step d.i) Fix initial investments for model at time t given optimal solution for time t-1
        models_d[t] = fix_initial_investments(models_d[t - 1],
            models_d[t],
            start_cap_d,
            inputs_d[t])

        # Step d.ii) Fix capacity tracking variables for endogenous retirements
        models_d[t] = fix_capacity_tracking(models_d[t - 1], models_d[t], cap_track_d, t)

        # Step d.iii) Solve the model at time t
        models_d[t], solve_time_d[t] = solve_model(models_d[t], setup)
        inputs_d[t]["solve_time"] = solve_time_d[t]
    end
    ##### END of final forward pass

    return models_d, stats_d, inputs_d
end

@doc raw"""
	fix_initial_investments(EP_prev::GenXModel, EP_cur::GenXModel, start_cap_d::Dict)

This function sets the right hand side values of the existing capacity linking constraints in the current stage $p$ to the realized values of the total available end capacity linking variable expressions from the previous stage $p-1$ as part of the forward pass.

inputs:

  * EP\_prev - JuMP model from the previous model stage $p-1$.
  * EP\_cur - JuMP model from the current model stage $p$.
  * start\_cap\_d – Dictionary which contains key-value pairs of available capacity investment expression names (as Symbols) as keys and their corresponding linking constraint names (as Symbols) as values.

returns: JuMP model with updated linking constraints.
"""
function fix_initial_investments(EP_prev::GenXModel,
        EP_cur::GenXModel,
        start_cap_d::Dict,
        inputs_d::Dict)
    ALL_CAP = union(inputs_d["RET_CAP"], inputs_d["NEW_CAP"]) # Set of all resources subject to inter-stage capacity tracking

    # start_cap_d dictionary contains the starting capacity expression name (e) as a key,
    # and the associated linking constraint name (c) as a value
    for (e, c) in start_cap_d
        for y in keys(EP_cur[c])
            # Set the right hand side value of the linking initial capacity constraint in the current stage to the value of the available capacity variable solved for in the previous stages
            if c == :cExistingTransCap
                set_normalized_rhs(EP_cur[c][y], value(EP_prev[e][y]))
            else
                if y[1] in ALL_CAP # extract resource integer index value from key
                    set_normalized_rhs(EP_cur[c][y], value(EP_prev[e][y]))
                end
            end
        end
    end
    return EP_cur
end

@doc raw"""
	fix_capacity_tracking(EP_prev::GenXModel, EP_cur::GenXModel, cap_track_d::Dict, cur_stage::Int)

This function sets the right hand side values of the new and retired capacity tracking linking constraints in the current stage $p$ to the realized values of the new and retired capacity tracking linking variables from the previous stage $p-1$ as part of the forward pass.
where tracking linking variables are defined variables for tracking, linking and passing realized expansion and retirement of capacities of each stage to the next stage.
Tracking linking variables are each defined in endogenous\_retirement\_discharge, endogenous\_retirement\_energy, and endogenous\_retirement\_charge functions. Three examples are "vCAPTRACK", "vCAPTRACKCHARGE", and ""vCAPTRACKENERGY"

inputs:

  * EP\_prev - JuMP model from the previous model stage $p-1$.
  * EP\_cur - JuMP model from the current model stage $p$.
  * cap\_track\_d – Dictionary which contains key-value pairs of capacity addition and retirement tracking variable names (as Symbols) as keys and their corresponding linking constraint names (as Symbols) as values.
  * cur\_period – Int representing the current model stage $p$.

returns: JuMP model with updated linking constraints.
"""
function fix_capacity_tracking(EP_prev::GenXModel,
        EP_cur::GenXModel,
        cap_track_d::Dict,
        cur_stage::Int)

    # cap_track_d dictionary contains the endogenous retirement tracking array variable name (v) as a key,
    # and the associated linking constraint name (c) as a value
    for (v, c) in cap_track_d

        # Tracking variables and constraints for retired capacity are named identicaly to those for newly
        # built capacity, except have the prefex "vRET" and "cRet", accordingly
        rv = Symbol("vRET", string(v)[2:end]) # Retired capacity tracking variable name (rv)
        rc = Symbol("cRet", string(c)[2:end]) # Retired capacity tracking constraint name (rc)

        for i in keys(EP_cur[c])
            i = i[1] # Extract integer index value from keys tuple - corresponding to generator index

            # For all previous stages, set the right hand side value of the tracking constraint in the current
            # stage to the value of the tracking constraint observed in the previous stage
            for p in 1:(cur_stage - 1)
                # Tracking newly buily capacity over all previous stages
                JuMP.set_normalized_rhs(EP_cur[c][i, p], value(EP_prev[v][i, p]))
                # Tracking retired capacity over all previous stages
                JuMP.set_normalized_rhs(EP_cur[rc][i, p], value(EP_prev[rv][i, p]))
            end
        end
    end

    return EP_cur
end

@doc raw"""
	add_cut(EP_cur::GenXModel, EP_next::GenXModel, start_cap_d::Dict, cap_track_d::Dict)

inputs:

  * EP\_cur - JuMP model from the current model stage $p$.
  * EP\_next - JuMP model from the next model stage $p+1$..
  * start\_cap\_d – Dictionary which contains key-value pairs of available capacity investment expression names (as Symbols) as keys and their corresponding linking constraint names (as Symbols) as values.
  * cap\_track\_d – Dictionary which contains key-value pairs of capacity addition and retirement tracking variable names (as Symbols) as keys and their corresponding linking constraint names (as Symbols) as values.

returns: JuMP expression representing a sum of Benders cuts for linking capacity investment variables to be added to the cost-to-go function.
"""
function add_cut(EP_cur::GenXModel, EP_next::GenXModel, start_cap_d::Dict, cap_track_d::Dict)
    next_obj_value = objective_value(EP_next) # Get the objective function value for the next investment planning stage

    eRHS = @expression(EP_cur, 0) # Initialize RHS of cut to 0

    # Generate cut components for investment decisions

    # start_cap_d dictionary contains the starting capacity expression name (e) as a key,
    # and the associated linking constraint name (c) as a value
    for (e, c) in start_cap_d

        # Continue if nothing to add to the cut
        if isempty(EP_next[e])
            continue
        end

        # Generate the cut component
        eCurRHS = generate_cut_component_inv(EP_cur, EP_next, e, c)

        # Add the cut component to the RHS
        eRHS = eRHS + eCurRHS
    end

    # Generate cut components for endogenous retirements.

    # cap_track_d dictionary contains the endogenous retirement tracking array variable name (v) as a key,
    # and the associated linking constraint name (c) as a value
    for (v, c) in cap_track_d

        # Continue if nothing to add to the cut
        if isempty(EP_next[c])
            continue
        end

        # Generate the cut component for new capacity
        eCurRHS_cap = generate_cut_component_track(EP_cur, EP_next, v, c)

        rv = Symbol("vRET", string(v)[2:end]) # Retired capacity tracking variable (rv)
        rc = Symbol("cRet", string(c)[2:end]) # Retired capacity tracking constraint (rc)

        # Generate the cut component for retired capacity
        eCurRHS_ret = generate_cut_component_track(EP_cur, EP_next, rv, rc)

        # Add the cut component to the RHS
        eRHS = eRHS + eCurRHS_cap + eCurRHS_ret
    end

    # Add the cut to the model
    @constraint(EP_cur, EP_cur[:vALPHA]>=next_obj_value - eRHS)

    return EP_cur
end

@doc raw"""
	generate_cut_component_inv(EP_cur::GenXModel, EP_next::GenXModel, expr_name::Symbol, constr_name::Symbol)

This function generates Bender's cut expressions for total new or retired capacity tracking linking variables in the form:
```math
\begin{aligned}
        \mu_{next}^{\top}(\hat{x}_{cur} - x_{cur})
\end{aligned}
```
where $\mu_{next}$ is a vector of dual values of the linking constraints defined by constr\_name in EP\_next, $\hat{x}_{cur}$ is a vector of realized values from the forward pass of the new or retired capacity tracking linking variables var\_name from EP\_cur, and $x_{cur}$ is a vector of unrealized new or retired capacity tracking linking variables from EP\_cur.

inputs:

  * EP\_cur - JuMP model from the current model stage $p$.
  * EP\_next - JuMP model from the next model stage $p+1$.
  * var\_name – Symbol representing the name of a JuMP variable array which contains total new or retired capacity tracking linking variables.
  * constr\_name – Symbol representing the name of the array of linking JuMP constraints which contain total new or retired capacity tracking linking variables.

returns: JuMP expression representing a sum of Benders cuts for linking capacity investment variables to be added to the cost-to-go function.
"""
function generate_cut_component_track(EP_cur::GenXModel,
        EP_next::GenXModel,
        var_name::Symbol,
        constr_name::Symbol)
    next_dual_value = Float64[]
    cur_inv_value = Float64[]
    cur_inv_var = []

    for k in keys(EP_next[constr_name])
        y = k[1] # Index representing resource
        p = k[2] # Index representing stage

        push!(next_dual_value, dual(EP_next[constr_name][y, p]))
        push!(cur_inv_value, value(EP_cur[var_name][y, p]))
        push!(cur_inv_var, EP_cur[var_name][y, p])
    end

    eCutComponent = @expression(EP_cur,
        dot(next_dual_value, (cur_inv_value .- cur_inv_var)))

    return eCutComponent
end

@doc raw"""
	generate_cut_component_inv(EP_cur::GenXModel, EP_next::GenXModel, expr_name::Symbol, constr_name::Symbol)

This function generates Bender's cut expressions for linking capacity investment variable expression in the form:
```math
\begin{aligned}
        \mu_{next}^{\top}(\hat{x}_{cur} - x_{cur})
\end{aligned}
```
where $\mu_{next}$ is a vector of dual values of the linking constraints defined by constr\_name in EP\_next, $\hat{x}_{cur}$ is a vector of realized values from the forward pass of the linking capacity investment variable expressions expr\_name from EP\_cur, and $x_{cur}$ is a vector of unrealized linking capacity investment variable expressions from EP\_cur. inputs:

inputs:

  * EP\_cur - JuMP model from the current model stage $p$, solved in the forward pass.
  * EP\_next - JuMP model from the next model stage $p+1$, solved in the forward pass.
  * expr\_name – Symbol representing the name of a JuMP expression array which contains linking capacity investment variables.
  * constr\_name – Symbol representing the name of the array of linking JuMP constraints which contain the linking capacity investment variables.

returns: JuMP expression representing a sum of Benders cuts for linking capacity investment variables to be added to the cost-to-go function.
"""
function generate_cut_component_inv(EP_cur::GenXModel,
        EP_next::GenXModel,
        expr_name::Symbol,
        constr_name::Symbol)
    next_dual_value = Float64[]
    cur_inv_value = Float64[]
    cur_inv_var = []

    for y in keys(EP_next[constr_name])
        push!(next_dual_value, dual(EP_next[constr_name][y]))
        push!(cur_inv_value, value(EP_cur[expr_name][y]))
        push!(cur_inv_var, EP_cur[expr_name][y])
    end

    eCutComponent = @expression(EP_cur,
        dot(next_dual_value, (cur_inv_value .- cur_inv_var)))

    return eCutComponent
end

@doc raw"""
	initialize_cost_to_go(settings_d::Dict, EP::GenXModel)

This function scales the model objective function so that costs are consistent with multi-stage modeling and introduces a cost-to-go function variable to the objective function.

The updated objective function $OBJ^{*}$ returned by this method takes the form:
```math
\begin{aligned}
    OBJ^{*} = DF * OPEXMULT * OBJ + \alpha
\end{aligned}
```
where $OBJ$ is the original objective function. $OBJ$ is scaled by two terms. The first is a discount factor (applied only in the non-myopic case), which discounts costs associated with the model stage $p$ to year-0 dollars:
```math
\begin{aligned}
    DF = \frac{1}{(1+WACC)^{\sum^{(p-1)}_{k=0}L_{k}}}
\end{aligned}
```
where $WACC$ is the weighted average cost of capital, and $L_{p}$ is the length of each stage in years (both set in multi\_stage\_settings.yml)

The second term is a discounted sum of annual operational expenses incurred each year of a multi-year model stage:
```math
\begin{aligned}
    & OPEXMULT = \sum^{L}_{l=1}\frac{1}{(1+WACC)^{l-1}}
\end{aligned}
```
Note that although the objective function contains investment costs, which occur only once and thus do not need to be scaled by OPEXMULT, these costs are multiplied by a factor of $\frac{1}{WACC}$ before being added to the objective function in investment\_discharge\_multi\_stage(), investment\_charge\_multi\_stage(), investment\_energy\_multi\_stage(), and transmission\_multi\_stage(). Thus, this step scales these costs back to their correct value.

The cost-to-go function $\alpha$ represents an approximation of future costs given the investment and retirement decisions in the current stage. It is constructed through the addition of cuts to the cost-to-go function $\alpha$ during the backwards pass.

inputs:

  * settings\_d - Dictionary containing settings dictionary configured in the multi-stage settings file multi\_stage\_settings.yml.
  * EP – JuMP model.

returns: JuMP model with updated objective function.
"""
function initialize_cost_to_go(settings_d::Dict, EP::GenXModel, inputs::Dict)
    cur_stage = settings_d["CurStage"] # Current DDP Investment Planning Stage
    cum_years = 0
    for stage_count in 1:(cur_stage - 1)
        cum_years += settings_d["StageLengths"][stage_count]
    end
    stage_len = settings_d["StageLengths"][cur_stage]
    wacc = settings_d["WACC"] # Interest Rate  and also the discount rate unless specified other wise
    myopic = settings_d["Myopic"] == 1 # 1 if myopic (only one forward pass), 0 if full DDP
    OPEXMULT = inputs["OPEXMULT"] # OPEX multiplier to count multiple years between two model stages, set in configure_multi_stage_inputs.jl

    # Overwrite the objective function to include the cost-to-go variable (not in myopic case)
    # Multiply discount factor to all terms except the alpha term or the cost-to-go function
    # All OPEX terms get an additional adjustment factor
    if myopic
        ### No discount factor or OPEX multiplier applied in myopic case as costs are left annualized.
        @objective(EP, Min, EP[:eObj])
    else
        DF = 1 / (1 + wacc)^(cum_years)  # Discount factor applied all to costs in each stage ###
        # Initialize the cost-to-go variable
        @variable(EP, vALPHA>=0)
        @objective(EP, Min, DF * OPEXMULT * EP[:eObj]+vALPHA)
    end

    return EP
end
