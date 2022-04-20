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

    v = 10.0
    θ = 0
    p = 0
    while true
        sleep(0)
        @return_if_told(EMG)
        meas = @fetch_or_continue(SENSE)
        speed = meas.speed
        heading = meas.heading
        segment = road.segments[road_segment(m,road)]
        

        if p < 4000
        cte, ctv = get_crosstrack_error(meas.position, meas.heading, meas.speed, 3, segment, road.lanes, road.lanewidth)
        p = p + 0.5
        else
            cte, ctv = get_crosstrack_error(meas.position, meas.heading, meas.speed, 1, segment, road.lanes, road.lanewidth)
        end
        K₁ = 0.5
        K₂ = 0.5
        δ = -K₁*cte-K₂*ctv

        
       # @try_update(SENSE, ego_meas)
       # @try_update(SENSE_FLEET, fleet_meas)
        err_1 = v-speed
        err_2 = clip(θ-heading, π/2)
        cmd = [0.0 max(min(δ, π/4.0), -π/4.0)]
        @replace(CMD, cmd)
    end
end
