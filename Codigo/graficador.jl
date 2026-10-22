
using OptimalControl
using NLPModelsIpopt
using Plots

### Funciones dadas
# Energía potencial (pendientes)
P(s) = 0
# Fuerza de tracción y frenado (break) máximos
gtr(s) = 100
gb(s) = 100
# fuerzas de disipación (cambiar factores a,b,c no negativos)
a,b,c = 0, 10, 10
w(K) = a + b*sqrt(K) + c*K
# "recovery factor" retorno de energía al frenar ∈[0,1]
α = 0.5

### Condiciones iniciales
E₀ = 0.01
Eₛ = 0.01
T = 1000 #?
S = 1000

ocp = @def begin
    t ∈ [0, S], time # en verdad es la variable espacial
    x ∈ R^2, state # x[1]: Energía kinética, E = K + P, por lo que necesitamos plantear Ṗ en la dinámica
                   # x[2]: tiempo, 
    u ∈ R^2, control

    x(0) == [E₀, 0]
    x(S) == [Eₛ, T]  # debo llegar con energía Eₛ en tiempo T

    # restricciones sobre el control
    u[1](t) ≥ 0
    u[1](t) - gtr( x[1](t)) ≤ 0

    u[2](t) ≥ 0
    u[2](t) - gb( x[1](t)) ≤ 0

    # Dinámica MAL IMPLEMENTADA HAY QUE SUMAR LA DERIVADA DE P
    ẋ(t) == [u[1](t) - u[2](t) - w(x[1](t)), 1/(sqrt(2x[1](t)))]
    ∫( u[1](t) - α*u[2](t)) → min
end

sol = solve(ocp)
plot(sol)
