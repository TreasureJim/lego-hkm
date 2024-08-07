using StaticArrays

struct JulietQueue
    queue::Array{UInt8, 1}
end

struct JointTarget
    j1::Float64
    j2::Float64
    j3::Float64
    j4::Float64
    j5::Float64
    j6::Float64
end

struct JointPos
    j1::Float64
    j2::Float64
    j3::Float64
    j4::Float64
    j5::Float64
    j6::Float64
    j7::Float64
    j8::Float64
    j9::Float64
    j10::Float64
    j11::Float64
    j12::Float64
    j13::Float64
    j14::Float64
    j15::Float64
end

struct MotionId
    id::SVector{16, UInt8}
    status::Int8
end

struct HKMPos
    j1::Float64
    j2::Float64
    j3::Float64
    j4::Float64
end

struct MotionMovJ
    target::JointTarget
    motion_id::SVector{16, UInt8}
end

struct Velocity
    percentage::Float64
    path_speed::Float64
    orientation_speed::Float64
    external_axis_l::Float64
    external_axis_j::Float64
    motion_id::SVector{16, UInt8}
end

struct Trigger
    trigger_id::SVector{16, UInt8}
end

struct RobTarget
    x::Float64
    y::Float64
    z::Float64
    a::Float64
    b::Float64
    c::Float64
end

struct RobotIOMessage
    handle::Int64
    val::Int8
end

struct OnParameter
    percentage::Float64
    time_ms::Float64
    trigger_id::SVector{16, UInt8}
end

struct OnDistance
    start_type::UInt16
    distance::Float64
    time_ms::Float64
    trigger_id::SVector{16, UInt8}
end

struct MoveLinear
    target::RobTarget
    motion_id::SVector{16, UInt8}
end

struct MoveArc
    apos::RobTarget
    target::RobTarget
    motion_id::SVector{16, UInt8}
end

struct MoveCircular
    apos::RobTarget
    target::RobTarget
    motion_id::SVector{16, UInt8}
end

struct MotionInfo
    nr_expected_params::Int64
    motion_id::SVector{16, UInt8}
end

struct KernelError
    error_id::UInt64
    error_level::UInt64
    error_info_ch::String
    error_info_en::String
    error_emergency::UInt64
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
