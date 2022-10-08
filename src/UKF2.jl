#= Use as State: 
    x = [p_x, p_y, p_z, Ψ, Θ]
    # The Roll angle is not used here as it is not relevant for position change
   Use as Input:
    u = [w_x, w_y, w_z, v, dt, steerAngle]
=#
function f(s::Vector{Float32}, u::Vector{Float32})
    Ψₜ = Ψ(s[4], u[5], u[6], β(u[6]), u[4])
    θₜ = θ_ang(s[5], u[5], [u[1], u[2], u[3]])
    ṡ = changeInPosition([u[1], u[2], u[3]], u[4], Ψₜ, θₜ, u[5], β=β(u[6]))
    p = s[1:3] + u[5]*ṡ

    return [p[1], p[2], p[3], Ψₜ, θₜ]
end


function prediction(Σₖ₋₁::Matrix{Float32}, uₜ::Vector{Float32}, P::Matrix{Float32}, p::PredictionSettings, wₘ::Vector{Float32}, wₖ::Vector{Float32})
    Χ = Matrix{Float32}(undef, n, 0)
    for i ∈ 0:2*n  Χ = hcat(Χ, f(Σₖ₋₁[:, i+1], uₜ)) end

    # generate matrix from weights
    W = (Matrix(I, 2*n+1, 2*n+1) .- wₘ)*(Matrix(I, 2*n+1, 2*n+1).*wₖ)*transpose((Matrix(I, 2*n+1, 2*n+1) .- wₘ))
    Pₖ = Χ * W * Χ' + p.processNoiseS*Matrix(I, size(P))

    xₖ = Χ*wₘ

    return Pₖ, xₖ, W, Χ
end

function correction(Zₖ::Matrix{Float32}, Χₖ::Matrix{Float32}, xₖ::Vector{Float32}, W::Matrix{Float32}, measurement::Vector{Float32}, wₘ::Vector{Float32}, ratedCC::Float32)
    Sₖ = Zₖ*W*Zₖ'
    Cₖ = Χₖ*W*Zₖ'

    K = Cₖ*inv(Sₖ + (ratedCC*p.measurementNoiseS+1)*Matrix(I, n, n))

    xₖ = xₖ + K*(measurement - Zₖ*wₘ)
    Pₖ = Pₖ - K*Sₖ*K'

    return xₖ, Pₖ
end

function generateSigmaPoints(x::Vector{Float32}, Pₖ₋₁::Matrix{Float32}, p::PredictionSettings)
    c = p.α^2*(n+p.κ)
    
    println(Pₖ₋₁)
    S = cholesky(forceHermetian(Pₖ₋₁)).U
    
    Σ = Matrix{Float32}(undef, n, 0)
    for i ∈ 0:n 
        Σ = hcat(Σ, x .+ sqrt(c)*(i==0 ? 0 : S[:, i])) 
    end
    for i ∈ 1:n 
        Σ = hcat(Σ, x - sqrt(c)*S[:, i]) 
    end

    return Σ
end

function computeWeights(mean::Bool, p::PredictionSettings)
    w = Vector{Float32}(undef, 0)
    λ = p.α^2*(n + p.κ) - n

    if mean push!(w, λ/(n+λ))
    else push!(w, λ/(n+λ) + (1-p.α^2+2)) end

    for i ∈ 1:2*n
        push!(w, 1/(2*(n+λ)))
    end 
    return w
end

function forceHermetian(m::Matrix{Float32})
    s = size(m)
    for x ∈ 1:s[1]
        for y ∈ 1:s[2]
            if x == y 
                continue 
            end

            if m[x,y] == m[y,x] continue end

            m[x,y] = m[y,x]
        end    
    end

    if !isposdef(m)
        println(m)
    end
    return m
end