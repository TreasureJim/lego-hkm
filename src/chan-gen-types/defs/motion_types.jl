using StaticArrays

"""
A struct representing the status of a command

`status`:
- error = -1
- in progress = 0
- complete = 1
"""
struct MotionId
    id::SVector{16, UInt8}
    status::Int8
end

struct RobTarget
    x::Float64
    y::Float64
    z::Float64
    j4::Float64
    a::Float64
    b::Float64
    c::Float64
end

struct HKMPos
    j1::Float64
    j2::Float64
    j3::Float64
    j4::Float64
end

struct MoveLinear
    motion_id::SVector{16, UInt8}
    target::RobTarget
end

struct MoveArc
    motion_id::SVector{16, UInt8}
    apos::RobTarget
    target::RobTarget
end

struct MoveCircular
    motion_id::SVector{16, UInt8}
    apos::RobTarget
    target::RobTarget
end

struct MoveJoint
    motion_id::SVector{16, UInt8}
    target::HKMPos
end

struct MovePos
    motion_id::SVector{16, UInt8}
    target::RobTarget
end

struct MoveJog
    motion_id::SVector{16, UInt8}
    x::Float64
    y::Float64
    z::Float64
    j1::Float64
    j2::Float64
    j3::Float64
    j4::Float64
end

struct BlendValue
    percentage::Float64
    distance::Float64
    vel_const::Float64
end

struct Blend
    blend_type::UInt16
    blend_value::BlendValue
    motion_id::SVector{16, UInt8}
end
