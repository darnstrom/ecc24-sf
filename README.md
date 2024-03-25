# ECC24 
This repository contains code to reproduce the experiments in the [ECC24](https://ecc24.euca-ecc.org/) contribution **Stealthy Deactivation of Safety Filters**.

## Generating results 
The code requires that [Julia](https://julialang.org/) is installed. If Julia is installed, run the following command in a terminal: 
```shell
julia --project=@. main.jl
```
If successful, the folder `result` will be generated with data that is reported in the manuscript. 

The notation used in the columns of the `.dat` files are
* `x`: the perceived state  
* `z`: the actual state  
* `u`: the applied control action 
* `hx`: the perceived safety margin
* `hz`: the actual safety margin
* `rho`: correlation metric of proposed detector 
