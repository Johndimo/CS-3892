function wrap(θ)
    mod(θ += pi, 2*pi) - pi
end

function clip(x, l)
    max(-l, min(l, x))
end

function keyboard_controller(KEY::Channel, 
                             CMD::Channel, 
                             SENSE::Channel, 
                             EMG::Channel;
                             K1=5, 
                             K2=.5, 
                             disp=false, 
                             V=0.0, 
                             θ = 0.0, 
                             V_max = 100.0, 
                             θ_step=0.1, 
                             V_step = 1.5)
    println("Keyboard controller in use.")
    println("Press 'i' to speed up,")
    println("Press 'k' to slow down,")
    println("Press 'j' to turn left,")
    println("Press 'l' to turn right.")

    while true
        sleep(0)
        @return_if_told(EMG)
        
        key = @take_or_default(KEY, ' ')
        meas = @fetch_or_continue(SENSE)

        speed = meas.speed
        heading = meas.heading
        segment = meas.road_segment_id
        if key == 'i'
            V = min(V_max, V+V_step)
        elseif key == 'j' 
            θ += θ_step
        elseif key == 'k'
            V = max(0, V-V_step)
        elseif key == 'l'
            θ -= θ_step
        end
        err_1 = V-speed
        err_2 = clip(θ-heading, π/2)
        cmd = [K1*err_1, K2*err_2]
        @replace(CMD, cmd)
        if disp
            print("\e[2K")
            print("\e[1G")
            @printf("Command: %f %f, speed: %f, segment: %d", cmd..., speed, segment)
        end
    end
end


function findLane(target_speed::Float64, meas::OracleMeas, fleet_meas::Dict{Int64, OracleMeas}, road, currLane)

    CMD_FLEET_SIM = Dict{Int, Channel{VehicleControl}}()
    CMD_EGO_SIM = Channel{VehicleControl}(1)
    
    mSim = Bicycle(state=[meas.position[1] meas.position[2] target_speed meas.heading], channel=CMD_EGO_SIM)
    currMovables = Dict(1=>mSim)
    for (id, mov) ∈ fleet_meas
        state = MVector{4, Float64}(mov.position[1], mov.position[2], mov.speed, 0.0)
        control = [0.0 mov.heading]
        width = 1.5 + 2.0 - 0.05
        length = 3.0 + 6.0 - 0.05
        height = 2.0 + 1.0 - 0.05
        color = parse(RGB, "rgb"*string(Tuple(rand(0:255,3))))
        channel = Channel{VehicleControl}(1)
        currMovables[id] = Bicycle(state=state,
                            control=control,
                            width=width,
                            lf=length/2,
                            lr=length/2,
                            height=height,
                            color=color,
                            target_vel=mov.target_vel,
                            target_lane=mov.target_lane,
                            channel=channel)
        CMD_FLEET_SIM[id] = channel
    end

    simIterations = 4
    Δ = 0.125
    dists = [0.0 0.0 0.0]
    vels = [0.0 0.0 0.0]

    lane1 = run_simulation_iteration(simIterations, Δ, currMovables, vels, 1, dists, road)
    lane2 = run_simulation_iteration(simIterations, Δ, currMovables, vels, 2, dists, road)
    lane3 = run_simulation_iteration(simIterations, Δ, currMovables, vels, 3, dists, road)

    canGo = [lane1 lane2 lane3]
    if dists[currLane] > 1
        canGo[currLane] = true
    end

    if currLane == 1 && dists[2] < 2
        dists[3] = 0
    elseif currLane == 3 && dists[2] < 2
        dists[1] = 0
    end
    
    temp = hcat(dists, vels)
    lanes = hcat(canGo, temp)
end

function controller(CMD::Channel, 
                    SENSE::Channel, 
                    SENSE_FLEET::Channel, 
                    EMG::Channel,
                    road,
                    m::Movable)
    local ego_meas
    local fleet_meas

    m.state[3] = 30.0
    prevLane = 1
    lanes = [0 0 0 0 0 0]
    canGo = [0 0 0]
    dists = [0 0 0]
    vels = [0.0 0.0 0.0]
    maxLane = 0
    segment = road.segments[road_segment(m,road)]
    laneImp = 1
    speedChange = 6
    while true
        sleep(0)
        @return_if_told(EMG)

        meas = @fetch_or_continue(SENSE)
        targetSpeed = meas.speed
        fleet_meas = @fetch_or_continue(SENSE_FLEET)
        foundALane = false

        if norm(meas.position - segment.center) < 104.5
            laneImp = 1
        elseif norm(meas.position - segment.center) < 109
            laneImp = 2
        else
            laneImp = 3
        end

        if dists[laneImp] < 2
            targetSpeed = vels[laneImp]
        end

        while !foundALane
            lanes = findLane(targetSpeed, meas, fleet_meas, road, prevLane)
            canGo = round.(Int64, lanes[1:3])
            dists = round.(Int64, lanes[4:6])
            vels = lanes[7:9]
            maxLane = findmax(dists)[2]
            
            if sum(canGo) > 0
                foundALane = true
            else
                if targetSpeed < 10
                    foundALane = true
                else
                    targetSpeed -= speedChange
                end
            end
        end
        
        targetLane = 0
        if laneImp == 2 || laneImp == maxLane
            targetLane = maxLane
        elseif laneImp == 1
            if dists[1] > dists[2]
                targetLane = 1
            else
                targetLane = 2
            end
        else
            if dists[3] > dists[2]
                targetLane = 3
            else
                targetLane = 2
            end
        end

        cte, ctv = get_crosstrack_error(meas.position, meas.heading, targetSpeed, targetLane, segment, road.lanes, road.lanewidth)
        prevLane = laneImp  

        K₁ = 0.04
        K₂ = 0.04
        δ = -K₁*cte-K₂*ctv

        if dists[laneImp] > 1
            targetSpeed += speedChange/2
        end

        targetSpeed = min(targetSpeed, 40)
        m.state[3] = targetSpeed
        cmd = [0 max(min(δ, π/4.0), -π/4.0)]

        @replace(CMD, cmd)
    end
end

function run_simulation_iteration(simIterations, Δ, movables, vels, laneNum, dists, road)
    lane = true
    movablesCopyLane = deepcopy(movables)
    K₁ = 0.04
    K₂ = 0.04
    max_dist = 7.5
    seg = road.segments[1]

    for i in 1:simIterations
        for (id, mov) ∈ movablesCopyLane
            pos1 = position(movablesCopyLane[1])
            if id != 1
                if mov.target_lane == laneNum
                    dist = norm(pos1 - position(mov))
                    if dist < max_dist || collision(movablesCopyLane[1], mov)
                        vels[laneNum] = speed(mov)
                        lane =  false
                        break;
                    end
                end
            end

            if id == 1
                cteS, ctvS = get_crosstrack_error(position(mov), heading(mov), speed(mov), laneNum, seg, road.lanes, road.lanewidth)
            else
                cteS, ctvS = get_crosstrack_error(position(mov), heading(mov), speed(mov), mov.target_lane, seg, road.lanes, road.lanewidth)
            end
            δ = -K₁*cteS-K₂*ctvS
            command = [0.0 max(min(δ, π/4.0), -π/4.0)]
            θ = mov.state[4]
            v = mov.state[3]
            a = command[1]
            ω = command[2]
            mov.state[1] += Δ * cos(θ) * v
            mov.state[2] += Δ * sin(θ) * v
            mov.state[3] += Δ * a
            mov.state[4] += Δ * ω
        end
        if !lane
            break;
        end
        dists[laneNum] += 1;
    end

    lane
end