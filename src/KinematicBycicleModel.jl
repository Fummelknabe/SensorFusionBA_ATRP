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

G = [1]