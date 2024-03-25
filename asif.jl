using JuMP
using DAQP
function asif_dint(udes,x;α=2)
    model = Model(DAQP.Optimizer)
    @variable(model, -1<=u<=1)
    @objective(model, Min, 0.5*((u-udes).^2))
    @constraint(model, c1, -2*x[2]*(1+u)+α*(-2*x[1]-x[2]^2) ≥ 0)
    optimize!(model);
    return value(u);
end
