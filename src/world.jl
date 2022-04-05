abstract type Segment end

Base.@kwdef struct StraightSegment <: Segment
    start::SVector{2, Float64}=SVector{2, Float64}(0,0)
    finish::SVector{2, Float64}=SVector{2, Float64}(0,100)
end

Base.@kwdef struct CurvedSegment <: Segment
    center::SVector{2, Float64}=SVector{2, Float64}(0,0)
    radius::Float64=100.0 # of inner lane boundary
    θ₁::Float64 # define arc length of circle
    θ₂::Float64 # θ₁ ≤ θ ≤ θ₂
end

struct Road
    segments::Vector{Segment} 
    lanewidth::Float64
    lanes::Int
end

function get_crosstrack_error(pos, heading, speed, target_lane, seg::StraightSegment, lanes, lanewidth)  
    del = seg.finish-seg.start
    del /= norm(del)
    perp = [del[2], -del[1]]
    ctv = -speed*perp'*[cos(heading), sin(heading)]

    #for l ∈ 1:lanes
    #    if pos'*perp ≤ seg.start'*perp + l*lanewidth
    #        return ((seg.start-pos)'*perp + (l-0.5)*lanewidth), ctv
    #    end
    #end
    #return (pos-(seg.start + (target_lane-0.5)*lanewidth*perp))'*perp, ctv
    return ((seg.start-pos)'*perp + (target_lane-0.5)*lanewidth), ctv
end

function get_crosstrack_error(pos, heading, speed, target_lane, seg::CurvedSegment, lanes, lanewidth)  
    rad = norm(pos-seg.center)
    if seg.θ₁ ≤ seg.θ₂
        perp = seg.center - pos
        perp /= norm(perp)
        ctv = speed*perp'*[cos(heading), sin(heading)]
        #for l ∈ 1:lanes
        #    if rad ≤ seg.radius + l*lanewidth
        #        return (seg.radius + (l-0.5)*lanewidth - rad), ctv
        #    end
        #end 
        return (seg.radius + (target_lane-0.5)*lanewidth - rad), ctv
    else
        perp = pos-seg.center
        perp /= norm(perp)
        ctv = speed*perp'*[cos(heading), sin(heading)]
        return rad - (seg.radius - (target_lane-0.5)*lanewidth), ctv
    end 
end

function road_segment(m, road; buf=5.0)
    if length(road.segments) == 1
        return 1
    end
    
    pos = position(m)
    
    for (i,seg) ∈ enumerate(road.segments)
        if isa(seg, StraightSegment)
            del = seg.finish-seg.start
            del /= norm(del)
            perp = [del[2], -del[1]]
            long = seg.start'*del-buf ≤ pos'*del ≤ seg.finish'*del
            #long = pos'*del ≤ seg.finish'*del
            lat = seg.start'*perp-buf ≤ pos'*perp ≤ seg.start'*perp + road.lanes*road.lanewidth + buf
            if lat && long
                return i
            end
        else
            if seg.θ₁ ≤ seg.θ₂
                rad = seg.radius-buf ≤ norm(pos-seg.center) ≤ seg.radius + road.lanes*road.lanewidth+buf
                angle = seg.θ₁-π/12.0 ≤ atan(pos[2]-seg.center[2], pos[1]-seg.center[1]) ≤ seg.θ₂
            else
                rad = seg.radius+buf ≥ norm(pos-seg.center) ≥ seg.radius - road.lanes*road.lanewidth-buf
                angle = seg.θ₂ ≤ atan(pos[2]-seg.center[2], pos[1]-seg.center[1]) ≤ seg.θ₁ + π/12.0
            end 
            if rad && angle
                return i
            end
        end
    end
    return length(road.segments)
end



function random_road(; length=5000.0, lanes=3, lanewidth=5.0)
    segments = []
    total_length = 0.0
    sign = 1.0
    θ = 0.0
    start = MVector{2, Float64}(0.0,0.0)
    while total_length < length
        r = rand()
        heading = [cos(θ), sin(θ)]
        if total_length < 1.0
            segment_length = 120.0
            straight = true
        else
            segment_length = 50.0+r*150.0
            straight = Bool(rand(0:1))
        end
        total_length += segment_length
        if straight
            finish = start + heading*segment_length
            push!(segments, StraightSegment(start=start, finish=finish))
            start .= finish
        else
            turn = r * (sign*pi - θ) + sign*π/8.0
            turn = max(min(π/2.0, turn),-π/2.0)
            radius = abs(segment_length / turn)
            perp = [-sin(θ), cos(θ)]
            center = start + sign*radius*perp


            θ₁ = atan(start[2]-center[2], start[1]-center[1])
            θ₂ = θ₁ + turn
            push!(segments, CurvedSegment(center, radius, θ₁, θ₂))
            θ += turn
            start .= center .+ radius*[cos(θ₂), sin(θ₂)]
            sign = -1.0*sign
        end
    end
    Road(segments, lanewidth, lanes)
end

function simple_loop(; radius=200.0, center=[0,radius], lanes=3, lanewidth=5.0)
    seg = CurvedSegment(center, radius, -π, π)
    Road([seg,], lanewidth, lanes)
end

function visualize_road(scene, road; res=1000) 
    for seg ∈ road.segments
        visualize_segment(scene, seg, road.lanes, road.lanewidth) 
    end
end

function visualize_segment(ax, seg::StraightSegment, lanes, lanewidth; res=1000, dashlen=5.0)
    len = norm(seg.finish-seg.start)

    units_per_dash = Int(floor(res/len*dashlen))
    starts = Vector(1:units_per_dash:res)
    starts = starts[1:2:end-1]

    t = Vector{Float64}(LinRange(0.0, len, res))
    θ = atan(seg.finish[2]-seg.start[2], seg.finish[1]-seg.start[1]) 
    z = zeros(res)

    x_line = seg.start[1] .+ cos(θ) * t
    y_line = seg.start[2] .+ sin(θ) * t
    lines!(ax, x_line, y_line, z, linewidth=3, color=:black)

    middle_dists = lanewidth * Vector(1:lanes-1)
    perp = [sin(θ), -cos(θ)]
    for dist ∈ middle_dists
        x_line = seg.start[1] + perp[1]*dist .+ cos(θ) * t
        y_line = seg.start[2] + perp[2]*dist .+ sin(θ) * t
        
        for s ∈ starts
            x_line[s+1:s+units_per_dash] .= NaN
            y_line[s+1:s+units_per_dash] .= NaN
        end
            
        lines!(ax, x_line, y_line, z, linewidth=3, color=:black)
    end
    
    x_line = seg.start[1] + perp[1]*lanes*lanewidth .+ cos(θ) * t
    y_line = seg.start[2] + perp[2]*lanes*lanewidth .+ sin(θ) * t
    lines!(ax, x_line, y_line, z, linewidth=3, color=:black)
end 

function visualize_segment(ax, seg::CurvedSegment, lanes, lanewidth; res=1000, dashlen=5.0)
    θ = LinRange(seg.θ₁, seg.θ₂, res)
    len = abs(seg.θ₁-seg.θ₂)*seg.radius
    
    units_per_dash = Int(floor(res/len*dashlen))
    starts = Vector(1:units_per_dash:res)
    starts = starts[1:2:end-1]
    c = cos.(θ)
    s = sin.(θ)
    z = zeros(res)
    if seg.θ₂ < seg.θ₁
        sign = -1.0
    else
        sign = 1.0
    end

    left_rad = seg.radius
    x_line = seg.center[1] .+ left_rad * c
    y_line = seg.center[2] .+ left_rad * s
    lines!(ax, x_line, y_line, z, linewidth=3, color=:black)

    middle_rads = sign*lanewidth * Vector(1:lanes-1) .+ seg.radius
    for rad ∈ middle_rads
        x_line = seg.center[1] .+ rad * c
        y_line = seg.center[2] .+ rad * s

        for s ∈ starts
            x_line[s+1:s+units_per_dash] .= NaN
            y_line[s+1:s+units_per_dash] .= NaN
        end
   
        lines!(ax, x_line, y_line, z, linewidth=3, color=:black)
    end
    right_rad = seg.radius + sign*lanewidth * lanes
    x_line = seg.center[1] .+ right_rad * c
    y_line = seg.center[2] .+ right_rad * s
    lines!(ax, x_line, y_line, z, linewidth=3, color=:black)
end
