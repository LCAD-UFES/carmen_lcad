# libtrailer_nlp
The tractor-trailer vehicle trajectory planning problem is formulated as an optimal control problem, which aims to minimize the total trajectory execution time, subject to the kinematic constraints of multi-articulated vehicles and boundary conditions.

![alt text](trailer_nlp_model.png)

## Setup

- The easiest way to install CASADI C++ API with IPOPT

```
pip3 install casadi==3.5.5
```

- Add the following lines to your bash profile

```
#offroad_planner
export CASADI_DIR=$(/usr/bin/env python3 -c "import casadi; print(casadi.__path__[0])")
export LD_LIBRARY_PATH=$CASADI_DIR:$LD_LIBRARY_PATH
```
