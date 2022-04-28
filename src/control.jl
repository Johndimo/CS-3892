function wrap(θ)
    mod(θ += pi, 2*pi) - pi
end

function clip(x, l)
    max(-l, min(l, x))
end

function keyboard_controller(KEY::ChannelLock, 
                             CMD::ChannelLock, 
                             SENSE::ChannelLock, 
                             EMG::ChannelLock;
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

    CMD_FLEET_SIM = Dict{Int, ChannelLock{VehicleControl}}()
    CMD_EGO_SIM = ChannelLock{VehicleControl}(1)
    
    mSim = Bicycle(state=[meas.position[1] meas.position[2] target_speed meas.heading], channel=CMD_EGO_SIM)
    currMovables = Dict(1=>mSim)
    for (id, mov) ∈ fleet_meas
        state = MVector{4, Float64}(mov.position[1], mov.position[2], mov.speed, 0.0)
        control = [0.0 mov.heading]
        width = 1.5 + 2.0 - 0.2
        length = 3.0 + 6.0 - 0.2
        height = 2.0 + 1.0 - 0.2
        color = parse(RGB, "rgb"*string(Tuple(rand(0:255,3))))
        channel = ChannelLock{VehicleControl}(1)
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

    Δ=0.125
    K₁ = 0.5
    K₂ = 0.5
    simIterations = 8
    max_dist = 12.5
    dists = [0 0 0]


    movablesCopyLane1 = deepcopy(currMovables)
        lane1 = true
        for i in 1:simIterations
            for (id, mov) ∈ movablesCopyLane1
                pos1 = position(movablesCopyLane1[1])
                if id != 1
                    if mov.target_lane == 1
                        dist = norm(pos1 - position(mov))
                        if dist < max_dist || collision(movablesCopyLane1[1], mov)
                            lane1 =  false
                            break;
                        end
                    end
                end

                seg = road.segments[road_segment(mov,road)]
                if id == 1
                    cteS, ctvS = get_crosstrack_error(position(mov), heading(mov), speed(mov), 1, seg, road.lanes, road.lanewidth)
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
            if !lane1
                break;
            end
            dists[1] += 1;
        end




        movablesCopyLane2 = deepcopy(currMovables)
        lane2 = true
        for i in 1:simIterations
            for (id, mov) ∈ movablesCopyLane2
                pos1 = position(movablesCopyLane2[1])
                if id != 1
                    if mov.target_lane == 2
                        dist = norm(pos1 - position(mov))
                        if dist < max_dist || collision(movablesCopyLane2[1], mov)
                            lane2 =  false
                            break;
                        end
                    end
                end

                seg = road.segments[road_segment(mov,road)]
                if id == 1
                    cteS, ctvS = get_crosstrack_error(position(mov), heading(mov), speed(mov), 1, seg, road.lanes, road.lanewidth)
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
            if !lane2
                break;
            end
            dists[2] += 1;
        end

        movablesCopyLane3 = deepcopy(currMovables)
        lane3 = true
        for i in 1:simIterations
            for (id, mov) ∈ movablesCopyLane3
                pos1 = position(movablesCopyLane3[1])
                if id != 1
                    if mov.target_lane == 3
                        dist = norm(pos1 - position(mov))
                        if dist < max_dist || collision(movablesCopyLane3[1], mov)
                            lane3 =  false
                            break;
                        end
                    end
                end

                seg = road.segments[road_segment(mov,road)]
                if id == 1
                    cteS, ctvS = get_crosstrack_error(position(mov), heading(mov), speed(mov), 3, seg, road.lanes, road.lanewidth)
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
            if !lane3
                break;
            end
            dists[3] += 1;
        end



        canGo = [lane1 lane2 lane3]
        if dists[currLane] > 3
            canGo[currLane] = true
        end

        lanes = hcat(canGo, dists)
end

function controller(CMD::ChannelLock, 
                    SENSE::ChannelLock, 
                    SENSE_FLEET::ChannelLock, 
                    EMG::ChannelLock,
                    road,
                    m::Movable)
    local ego_meas
    local fleet_meas

    θ = 0
    p = 0
    v = 30.0
    t = 50
    m.state[3] = v
    prevLane = 1
    lanes = [0 0 0 0 0 0]
    canGo = [0 0 0]
    dists = [0 0 0]
    maxLane = 0
    while true
        sleep(0)
        @return_if_told(EMG)

        meas = @fetch_or_continue(SENSE)
        segment = road.segments[road_segment(m,road)]
        targetSpeed = meas.speed
        fleet_meas = @fetch_or_continue(SENSE_FLEET)

        
        foundALane = false
        speedChange = 5
        while !foundALane
            lanes = findLane(targetSpeed, meas, fleet_meas, road, prevLane)
            canGo = lanes[1:3]
            dists = lanes[4:6]
            maxLane = findmax(dists)[2]
            
            if sum(canGo) > 0
                foundALane = true
            else
                if targetSpeed < 15
                    foundALane = true
                else
                    targetSpeed -= speedChange
                end
            end
        end

        
        targetLane = 0

        
        
        
        if prevLane == 2
            targetLane = maxLane
        elseif prevLane == 1
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

        if dists[targetLane] < 3
            targetLane = prevLane
            targetSpeed = 10
        end

        if dists[targetLane] > 1
            targetSpeed += speedChange/1.2
        end




        targetSpeed = min(targetSpeed, 35.0)

        cte, ctv = get_crosstrack_error(meas.position, meas.heading, targetSpeed, targetLane, segment, road.lanes, road.lanewidth)
        prevLane = targetLane

        K₁ = 0.05
        K₂ = 0.05
        δ = -K₁*cte-K₂*ctv

    # @try_update(SENSE, ego_meas)
    # @try_update(SENSE_FLEET, fleet_meas)
        err_1 = v-meas.speed
        err_2 = clip(θ-meas.heading, π/2)
        accel = 50*err_1
        m.state[3] = targetSpeed
        cmd = [0 max(min(δ, π/4.0), -π/4.0)]

        @replace(CMD, cmd)
    end
end