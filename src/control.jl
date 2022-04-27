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
    v = 30
    m.state[3] = v
    while true
        sleep(0)
        @return_if_told(EMG)
        meas = @fetch_or_continue(SENSE)
        fleet_meas = @fetch_or_continue(SENSE_FLEET)
        segment = road.segments[road_segment(m,road)]

        CMD_FLEET_SIM = Dict{Int, ChannelLock{VehicleControl}}()
        CMD_EGO_SIM = ChannelLock{VehicleControl}(1)
        
        mSim = Bicycle(state=[meas.position[1] meas.position[2] meas.speed meas.heading], channel=CMD_EGO_SIM)
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
         


        Δ=0.25
        K₁ = 0.5
        K₂ = 0.5
        simIterations = 5
        max_dist = 15.0
        lane1dist = 10000
        lane2dist = 10000
        lane3dist = 10000
        
        

        movablesCopyLane1 = deepcopy(currMovables)
        lane1 = true
        for i in 1:simIterations
            for (id, mov) ∈ movablesCopyLane1
                pos1 = position(movablesCopyLane1[1])
                if id != 1
                    if mov.target_lane == 1
                        dist = norm(pos1 - position(mov))
                        if dist < max_dist && collision(movablesCopyLane1[1], mov)
                            lane1dist = i*Δ 
                            lane1 =  false
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
                            lane1dist = i*Δ 
                            lane2 =  false
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
            
        end

        lanes = [lane1 lane2 lane3]

        
        if lane3
            cte, ctv = get_crosstrack_error(meas.position, meas.heading, meas.speed, 3, segment, road.lanes, road.lanewidth)
        elseif lane2
            cte, ctv = get_crosstrack_error(meas.position, meas.heading, meas.speed, 2, segment, road.lanes, road.lanewidth)
        elseif lane1
            cte, ctv = get_crosstrack_error(meas.position, meas.heading, meas.speed, 1, segment, road.lanes, road.lanewidth)
        else
            #change speed
            cte, ctv = get_crosstrack_error(meas.position, meas.heading, meas.speed, 3, segment, road.lanes, road.lanewidth)
        end

        K₁ = 0.05
        K₂ = 0.05
        δ = -K₁*cte-K₂*ctv

    # @try_update(SENSE, ego_meas)
    # @try_update(SENSE_FLEET, fleet_meas)
        err_1 = v-meas.speed
        err_2 = clip(θ-meas.heading, π/2)
        accel = 50*err_1
        cmd = [0 max(min(δ, π/75.0), -π/75.0)]
        @replace(CMD, cmd)
    end
end