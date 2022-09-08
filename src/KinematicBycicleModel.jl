# This model cannot desribe change in z position
# Just movement on 2D plane (horizontal plane)
# Measure again!
const lᵥ = 0.59
const lₕ = 0.10

# important equations (velocity is omitted as acceleration is not useful)
ẋ(v, Ψ, β) = v*cos(Ψ + β)
ẏ(v, Ψ, β) = v*sin(Ψ + β)
#ż(v, Ψ, β) = 
δΨ(v, β, δ) = v/(lₕ+lᵥ)*cos(β)*tan(δ)
β(δ) = atan(lₕ/(lᵥ+lₕ)*tan(δ))

function f(sₜ₋₁::PositionalState, uₜ::PositionalData, v::Float32) 
    βₜ = β(uₜ.steerAngle*π/180)
    Ψ = sₜ₋₁.Ψ + uₜ.deltaTime*δΨ(v, βₜ, uₜ.steerAngle*π/180)
    x = sₜ₋₁.position[1] + uₜ.deltaTime*ẋ(v, Ψ, βₜ)
    y = sₜ₋₁.position[2] + uₜ.deltaTime*ẏ(v, Ψ, βₜ)
    z = sₜ₋₁.position[3] + uₜ.deltaTime*ż(v, Ψ, βₜ)

    return Float32.([x, y, Ψ])
end

function f(s::Vector{Float32}, u::Vector{Float32})

end

"""
This performs the whole prediction step for the UKF.

# Arguments
`μₜ₋₁::Vector{Float32}`: The previous mean e.g. state.
`wₘ::Vector{Float32}`: The weights for the mean.
`wₖ::Vector{Float32}`: The weights for the covariance.
`Χₜ₋₁::Vector{Vector{Float32}}`: The previous Sigma points.
`uₜ::Vector{Float32}`: The input for the system.
`Σₜ₋₁::Matrix{Float32}`: The covariance of previous estimation.
`p::PredictionSettings`: Parameters to influence estimation.
"""
function UKF_prediction(μₜ₋₁::Vector{Float32}, wₘ::Vector{Float32}, wₖ::Vector{Float32}, Χₜ₋₁::Vector{Vector{Float32}}, uₜ::Vector{Float32}, Σₜ₋₁::Matrix{Float32}, p::PredictionSettings)
    μₜ̇ = sum(wₘ[i+1]*f(Χₜ₋₁[i+1], uₜ) for i ∈ 0:2*p.n)

    Χₜ = generateSigmaPoints(μₜ₋₁, Σₜ₋₁, wₘ, p)

    Σₜ = sum(wₖ[i+1]*(f(Χₜ₋₁, uₜ) - μₜ̇)*(f(Χₜ₋₁, uₜ) - μₜ̇)' for i ∈ 0:2*n) + p.process_noise_s # not sure if this should be added inside of sum

    return μₜ̇, Χₜ, Σₜ
end

function UKF_update(μₜ̇::Vector{Float32}, wₘ::Vector{Float32}, wₖ::Vector{Float32}, Χₜ::Vector{Vector{Float32}}, Σₜ̇::Matrix{Float32}, p::PredictionSettings, measurement::Vector{Float32})
    Zₜ = Matrix{Float32}(undef, length(μₜ̇), size(Χₜ)[2])
    for i ∈ 1:size(Χₜ)[2]
        Zₜ = hcat(Zₜ, Hₛ*Χₜ[:, i])
    end

    zₜ = sum(wₘ[i+1]*Zₜ[:, i] for i ∈ 0:2*p.n)

    Sₜ = sum(wₖ[i+1]*(Zₜ[:, i] - zₜ)*(Zₜ[:, i] - zₜ)' for i ∈ 0:2*n) + p.measurementNoiseS

    # calculate Kalman gain
    Kₜ = sum(wₖ[i+1]*(Χₜ[i+1] - μₜ̇)*(Zₜ[:, i] - zₜ)' for i ∈ 0:2*n) * Sₜ^-1

    μₜ = μₜ̇ + Kₜ*(measurement - zₜ)
    Σₜ = Σₜ̇ - Kₜ*Sₜ*Kₜ'
    return μₜ, Σₜ
end

"""
Compute the weights for the unscented transform.

# Arguments
- `mean::Bool`: If true computes weights for mean, if not for covariance.
- `params::PredictionSettings`: Parameters to influence estimation.
"""
function weights(mean::Bool, p::PredictionSettings)
    w = Vector{Float32}(undef, 0)

    if mean push!(w, p.λ/(p.n+p.λ))
    else push!(w, p.λ/(p.n+p.λ) + (1-p.α^2+2))

    for i ∈ 1:2*n
        push!(w, 1/(2*(p.n+p.λ)))
    end 
    return w
end

"""
Generate Sigma points from previous mean and covariance.

# Arguments
- `μₜ::Vector{Float32}`: The previous state containing the position and θ, Ψ
- `Σₜ::Matrix{Float32}`: The covariance of previous estimation.
- `params::PredictionSettings`: Parameters to influence estimation.
"""
function generateSigmaPoints(μₜ₋₁::Vector{Float32}, Σₜ₋₁::Matrix{Float32}, p::PredictionSettings)
    # Define vector holding sigma points
    χ = Vector{Vector{Float32}}(undef, 0)

    # Add the last mean
    push!(Χ, μₜ₋₁)

    # Add the remaining sigma points spread around mean
    for i ∈ 1:n
        push!(Χ, μₜ₋₁ + sqrt((p.n + p.λ) * Σₜ₋₁)[:, i])
    end

    for i ∈ (n+1):2*n
        push!(Χ, μₜ₋₁ - sqrt((p.n + p.λ) * Σₜ₋₁)[:, i])
    end
end
