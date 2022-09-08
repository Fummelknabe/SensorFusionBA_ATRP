# This model cannot desribe change in z position
# Just movement on 2D plane (horizontal plane)
# Measure again!
const lᵥ = 0.59
const lₕ = 0.10

# important equations (velocity is omitted as acceleration is not useful)
ẋ(v, Ψ, β) = v*cos(Ψ + β)
ẏ(v, Ψ, β) = v*sin(Ψ + β)
δΨ(v, β) = v/lₕ*sin(β)
β(δ) = atan(lₕ/(lᵥ+lₕ)*tan(δ))

function f(sₜ₋₁::State, uₜ::Input) 
    βₜ = β(uₜ.δ)
    Ψ = sₜ₋₁.Ψ + uₜ.δt*δΨ(uₜ.v, βₜ)
    x = sₜ₋₁.x + uₜ.δt*ẋ(uₜ.v, Ψ, βₜ)
    y = sₜ₋₁.y + uₜ.δt*ẏ(uₜ.v, Ψ, βₜ)

    return Float32.([x, y, Ψ])
end

# After linearization and discretization
F(δt, v) = [1 0 0;
            0 1 δt*v;
            1 0 0]
G(δt, v) = [0, δt*v, δt*v/lₕ]