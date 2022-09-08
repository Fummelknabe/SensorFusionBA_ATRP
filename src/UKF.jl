#= Use as State: 
    x = [p_x, p_y, p_z, Ψ, Θ]
   Use as Input:
    u = [a_x, a_y, a_z, v, dt, steerAngle]
=#
function f(s::Vector{Float32}, u::Vector{Float32})
    Ψₜ = Ψ(s[4], u[5], u[6], β(u[6]), u[4])
    θₜ = θ_acc(s[5], u[5], [u[1], u[2], u[3]])
    ṡ = changeInPosition([u[1], u[2], u[3]], u[4], Ψₜ, θₜ, u[5], β=β(u[6]))
    p = s[1:3] + u[5]*ṡ

    return [p[1], p[2], p[3], Ψₜ, θₜ]
end

"""
This performs the whole prediction step for the UKF.

# Arguments
- `μₜ₋₁::Vector{Float32}`: The previous mean e.g. state.
- `wₘ::Vector{Float32}`: The weights for the mean.
- `wₖ::Vector{Float32}`: The weights for the covariance.
- `Χₜ₋₁::Vector{Vector{Float32}}`: The previous Sigma points.
- `uₜ::Vector{Float32}`: The input for the system.
- `Σₜ₋₁::Matrix{Float32}`: The covariance of previous estimation.
- `p::PredictionSettings`: Parameters to influence estimation.

# Returns
- `μₜ̇`: Predicted state.
- `Χₜ`: Next Sigma Points.
- `Σₜ`: New covariance.
"""
function UKF_prediction(μₜ₋₁::Vector{Float32}, wₘ::Vector{Float32}, wₖ::Vector{Float32}, Χₜ₋₁::Vector{Vector{Float32}}, uₜ::Vector{Float32}, Σₜ₋₁::Matrix{Float32}, p::PredictionSettings)
    #println("DEBUG - dimensions: mean: $(size(μₜ₋₁)), weight m: $(size(wₘ)), weight k: $(size(wₖ)), sigma points: $(size(Χₜ₋₁)), u: $(size(uₜ)), sigma: $(size(Σₜ₋₁))")
    μₜ̇ = sum(wₘ[i+1]*f(Χₜ₋₁[i+1], uₜ) for i ∈ 0:2*p.n)
    #println("DEBUG - mean: $(μₜ̇)")

    Χₜ = generateSigmaPoints(μₜ₋₁, Σₜ₋₁, p)
    #println("DEBUG - sigma points: $(Χₜ)")

    Σₜ = sum(wₖ[i+1]*(f(Χₜ₋₁[i+1], uₜ) - μₜ̇)*(f(Χₜ₋₁[i+1], uₜ) - μₜ̇)' for i ∈ 0:2*p.n) + p.processNoiseS*Matrix(I, size(Σₜ₋₁)) # not sure if this should be added inside of sum
    #println("DEBUG - covariance: $(Σₜ)")
    #println("DEBUG - weights m: $(wₘ)")
    #println("DEBUG - weight k: $(wₖ)")

    return μₜ̇, Χₜ, Σₜ
end


"""
Update step of the unscented Kalman Filter.

# Returns
- `μₜ`: The new mean state.
- `Σₜ`: The new covariance.
"""
function UKF_update(μₜ̇::Vector{Float32}, wₘ::Vector{Float32}, wₖ::Vector{Float32}, Χₜ::Vector{Vector{Float32}}, Σₜ̇::Matrix{Float32}, p::PredictionSettings, measurement::Vector{Float32})
    l = length(μₜ̇)
    Zₜ = Matrix{Float32}(undef, l, length(Χₜ))
    for i ∈ 1:length(Χₜ)
        Zₜ = hcat(Zₜ, Hₛ*Χₜ[i])
    end
    Zₜ[isnan.(Zₜ)] .= 0.0

    zₜ = sum(wₘ[i+1]*Zₜ[:, i+1] for i ∈ 0:2*p.n)
    zₜ[isnan.(zₜ)] .= 0.0

    Sₜ = sum(wₖ[i+1]*(Zₜ[:, i+1] - zₜ)*(Zₜ[:, i+1] - zₜ)' for i ∈ 0:2*p.n) + p.measurementNoiseS*Matrix(I, l, l)
    Sₜ[isnan.(Sₜ)] .= 0.0
    println("DEBUG - S: $(Sₜ)")
    println("DEBUG - Z: $(Zₜ)")
    println("DEBUG - z: $(zₜ)")

    # calculate Kalman gain
    Kₜ = sum(wₖ[i+1]*(Χₜ[i+1] - μₜ̇)*(Zₜ[:, i+1] - zₜ)' for i ∈ 0:2*p.n) * Sₜ^-1

    μₜ = μₜ̇ + Kₜ*(measurement - zₜ)
    Σₜ = abs.(Σₜ̇ - Kₜ*Sₜ*Kₜ')
    #println("DEBUG - new mean: $(μₜ)")
    #println("DEBUG - new covariance: $(Σₜ)")
    return μₜ, Σₜ
end

"""
Compute the weights for the unscented transform.

# Arguments
- `mean::Bool`: If true computes weights for mean, if not for covariance.
- `params::PredictionSettings`: Parameters to influence estimation.
"""
function computeWeights(mean::Bool, p::PredictionSettings)
    w = Vector{Float32}(undef, 0)

    if mean push!(w, p.λ/(p.n+p.λ))
    else push!(w, p.λ/(p.n+p.λ) + (1-p.α^2+2)) end

    for i ∈ 1:2*p.n
        push!(w, 1/(2*(p.n+p.λ)))
    end 
    return w
end

"""
Generate Sigma points from previous mean and covariance.

# Arguments
- `μₜ::Vector{Float32}`: The previous state containing the position and θ, Ψ
- `Σₜ::Matrix{Float32}`: The covariance of previous estimation.
- `p::PredictionSettings`: Parameters to influence estimation.
"""
function generateSigmaPoints(μₜ₋₁::Vector{Float32}, Σₜ₋₁::Matrix{Float32}, p::PredictionSettings)
    # Define vector holding sigma points
    Χ = Vector{Vector{Float32}}(undef, 0)

    # Add the last mean
    push!(Χ, μₜ₋₁)

    # Add the remaining sigma points spread around mean
    for i ∈ 1:p.n
        push!(Χ, μₜ₋₁ + sqrt((p.n + p.λ) * Σₜ₋₁)[:, i])
    end

    for i ∈ (p.n+1):2*p.n
        push!(Χ, μₜ₋₁ - sqrt((p.n + p.λ) * Σₜ₋₁)[:, i])
    end
    return Χ
end
