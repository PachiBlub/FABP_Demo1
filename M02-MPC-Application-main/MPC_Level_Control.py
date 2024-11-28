import numpy as np
import sys
from casadi import *
import os
rel_do_mpc_path = os.path.join('..','..','..')
sys.path.append(rel_do_mpc_path)
import do_mpc
import matplotlib.pyplot as plt
from opcua import Client
import opcua.ua as ua
from time import sleep

# Define MPC_LevelC
class MPC_LevelC:

    def __init__(self):
        self.client_m02=Client("opc.tcp://10.6.51.20:4840")
        self.client_m02.session_timeout = 30000
        self.client_m02.connect()

        self.Inlet_Valve = self.client_m02.get_node('ns=4;i=4')  # VVo3 - Inlet valve
        self.Outlet_Valve = self.client_m02.get_node('ns=4;i=2')  # M02 VV3010 - Outlet valve
        self.Level_raw = self.client_m02.get_node('ns=4;i=7') #LI2021 - Level sensor

    def Calc_Level(self):
        Level_raw_value=self.Level_raw.get_value()
        m = 100 / (23600 + 358)
        b = m * 358
        calculated_level = round(m * Level_raw_value + b)/1000 #level in meters
        return calculated_level
    
    def Open_Inlet(self):
        self.Inlet_Valve.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
        print("opend inlet")
        
    def Close_Inlet(self):
        self.Inlet_Valve.set_attribute(ua.AttributeIds.Value, ua.DataValue(False))
        print("closed inlet")

    def Open_Outlet(self):
        self.Outlet_Valve.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
        print("opend Outlet")

    def Close_Outlet(self):
        self.Outlet_Valve.set_attribute(ua.AttributeIds.Value, ua.DataValue(False))
        print("Closed outlet")

# Set up MPC model
model_type = 'continuous'          # either 'discrete' or 'continuous'
model = do_mpc.model.Model(model_type)

# States struct (optimization variables):
level = model.set_variable(var_type='_x', var_name='level')

# Input struct (optimization variables):
S_vv04 = model.set_variable(var_type='_u', var_name='S_vv04')
S_vv02 = model.set_variable(var_type='_u', var_name='S_vv02')

# Calculation parameters in SI units
A = 0.0001767146 # Cross-sectional area # m^2
k_vv04 = 0.00015 # Valve constant, valve 4 # m^3/s    #negative sign for the outflow
k_vv02 = 0.00015 # Valve constant, valve 2# m^3/s  
P_src = 400000 # Pressure, water source # N/m^2
P_env = 100000 # Pressure, water source # N/m^2
Ro= 1000 # Density, water # kg/m^3
g = 9.81 #Gravitational acceleration #m/s^2

# Set right-hand side
model.set_rhs('level', ((S_vv02*k_vv02)-(S_vv04*k_vv04))/(A*100))

# Set up MPC Controller
model.setup()
mpc = do_mpc.controller.MPC(model)
setup_mpc = {
    'n_horizon': 10,
    't_step': 0.1,
    #Use MA27 linear solver in ipopt for faster calculations:
    #'nlpsol_opts': {'ipopt.linear_solver': 'MA27'}
}

mpc.set_param(**setup_mpc)

#Scaling
mpc.scaling['_x', 'level'] = 1000
mpc.scaling['_u', 'S_vv04'] = 1000
mpc.scaling['_u', 'S_vv02'] = 1000

#Objective
setpoint = 0.010 # setpoint for the target level in meters
_x = model.x
mterm = (_x['level'] - setpoint)**2 # terminal cost    
lterm = (_x['level'] - setpoint)**2 # stage cost
mpc.set_objective(mterm=mterm, lterm=lterm)

# input penalty
mpc.set_rterm(S_vv04=10, S_vv02 = 10)   #find the correct values 

#Constraints
# lower bounds of the state               
#mpc.bounds['lower', '_x', 'level'] = 0

# upper bounds of the state
#mpc.bounds['upper', '_x', 'level'] = 0.2

# lower bounds of the inputs
mpc.bounds['lower', '_u', 'S_vv04'] = 0
mpc.bounds['lower', '_u', 'S_vv02'] = 0

# upper bounds of the inputs
mpc.bounds['upper', '_u', 'S_vv04'] = 1
mpc.bounds['upper', '_u', 'S_vv02'] = 1

mpc.setup()
M = MPC_LevelC()

# Set the initial state of mpc
level_0 = np.array(M.Calc_Level()) # This is the initial level of the tank
#level_0 = np.array(0.1)
x0 = np.array(level_0)
mpc.x0 = x0
mpc.set_initial_guess()

# Run the MPC
M.Close_Inlet()
M.Close_Outlet()
i=1
while M.Calc_Level()!=setpoint:
    u0 = mpc.make_step(x0)
    print("MPC_It:", i)
    u0_array = np.array(u0)
    print("MPC Result:", u0_array)  # Prints the control actions for S_vv04 and S_vv02 
    
    
    #As the valve is binary we have to change the continuos amounts to binary ones.
    if u0_array[0]> 0.8:
        u0_array[0] = 1
        M.Open_Outlet()
        
    elif u0_array[0] < 0.8:
        u0_array[0] = 0
        M.Close_Outlet()
    else:
        print("Discretization error")

    if u0_array[1] > 0.8:
        u0_array[1] = 1
        M.Open_Inlet()
    elif u0_array[1] < 0.8:
        u0_array[1] = 0
        M.Close_Inlet()
    else:
        print("Discretization error")

    # Print the modified u0_array_act
    print("Discretization Result:", u0_array)


    x0 = np.array(M.Calc_Level()) #  measerued real values
    print("Measured Level:", x0)
    i=i+1

M.Close_Inlet()
M.Close_Outlet()
print("MPC finished")
   