## Init
using Pkg
Pkg.instantiate()
include("asif.jl")
include("examples.jl")
include("observer.jl")
using Plots, LinearAlgebra, DelimitedFiles

## Setup problem and attack policy 
f,g,h = double_int()
x0 = [-1.75;0];
Ts, T = 0.01, 3; 
N = Int(T/Ts)

A,B = [1 Ts; 0 1],[0; Ts]   # Linearized model used for stationary Kalman filter
Q = diagm([1.0,1]) # Process noise

δ = 0.001;

grad(x) = [-2;-2*x[2]] # Gradient of safety margin 


function gradient_attack_exp(x̂,grad,h,K;δ=0.001)
    if(norm(K'*grad(x̂)) > 1e-6)
        y = h(x̂)+δ*normalize(K'*grad(x̂));
    else
        y = h(x̂)
    end
end

## Simulate closed loop system 
for advers_type in [:none,:random,:gradient1dof, :gradient2dof]
    x,xhat,xhat_per,z = x0,x0,x0,x0;
    udes = 1;
    X,Z,U = zeros(2,N), zeros(2,N),zeros(N);
    As = zeros(N)
    Rhos = zeros(N);

    # Setup measruement equation + Kalman gain depending on if ny=1 or ny=2
    if(advers_type==:gradient1dof)
        C,R,Rs = [1.0 0],diagm([.001]), zeros(1,N);
    else # measure both states
        C,R,Rs = [1.0 0; 0 1],diagm([.001,0.001]), zeros(2,N);
    end

    K = stationary_kalman(A,C,R,Q)
    hm(x) = C*x # Measurement function

    # Start simulation 
    for i = 1:N 
        # Measurement
        y = C*x + R*randn(size(C,1)) # Get measurement
        xhat,_ = correct(xhat,y,C,K) # Correct measurments, adversary 

        # False data injection 
        if(advers_type == :none)
            yadv = y
        elseif(advers_type == :random)
            yadv = hm(xhat_per)+δ*normalize(randn(2))
        else # Gradient attack
            yadv = gradient_attack_exp(xhat_per,grad,hm,K;δ=0.001)
        end
        xhat_per,r = correct(xhat_per,yadv,C,K) # Correct measurements, controller

        # Safety filter
        u = asif_dint(udes[1],xhat_per) # Compute "safe" control

        # Log information
        X[:,i],U[i],Rs[:,i],As[i] = x,u,r[:],udes[1];
        Rhos[i] = (grad(xhat_per)'*K*r[:])/(δ*norm(K'*grad(xhat_per),2));
        Z[:,i] = xhat_per;

        # Time update 
        x += Ts*(f(x)+g(x)*u) # Step system
        xhat = predict(xhat,u,A,B) # Prediction step, adversary
        xhat_per = predict(xhat_per,u,A,B) # Prediction step, controller
    end
    # Write to file gradient _new 
    isdir("result") || mkdir("result");
    open("result/"*string(advers_type)*".dat"; write=true) do f
        write(f, "t x1 x2 z1 z2 u hx hz rho\n")
        writedlm(f, [collect(0:Ts:T-Ts) X[1,:] X[2,:] Z[1,:] Z[2,:] U [h(X[:,i]) for i in 1:size(Z,2)] [h(Z[:,i]) for i in 1:size(Z,2)] Rhos])
    end
end
