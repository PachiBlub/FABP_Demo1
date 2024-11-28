import numpy as np
import sys
from casadi import *
import os
rel_do_mpc_path = os.path.join('..','..','..')
sys.path.append(rel_do_mpc_path)
import importlib.util 
import do_mpc
import matplotlib.pyplot as plt
from opcua import Client, Server
import opcua.ua as ua
import time
import importlib.util

# Define OPC UA Client
class Control_Client:

    #set up client
    def __init__(self):
        self.client_m02=Client("opc.tcp://10.6.51.20:4840")
        self.client_m02.session_timeout = 30000
        self.client_m02.connect()

        self.Inlet_Valve = self.client_m02.get_node('ns=4;i=4')  # VVo3 - Inlet valve
        self.Outlet_Valve = self.client_m02.get_node('ns=4;i=2')  # M02 VV3010 - Outlet valve
        self.Level_raw = self.client_m02.get_node('ns=4;i=7') #LI2021 - Level sensor

    
    # get data from M02
    def ReadFromM02(self):
        Level_raw_value=self.Level_raw.get_value()
        print("M02 data read")
        return Level_raw_value

    # write data to M02
    def WriteToM02(self, values):
        self.Inlet_Valve.set_attribute(ua.AttributeIds.Value, ua.DataValue(values[0]))
        self.Outlet_Valve.set_attribute(ua.AttributeIds.Value, ua.DataValue(values[1]))
        print("Data written to M02")



# Define OPC UA Server
class Control_Server:
    
   # set up server
   def __init__(self):
      self.server = Server()
      self.server.set_endpoint("opc.tcp://0.0.0.0:4841") #"opc.tcp://10.6.51.100:4840")
      
      # setup our own namespace, not really necessary but should as spec
      uri = "http://examples.freeopcua.github.io"
      idx = self.server.register_namespace(uri)

      self.objects = self.server.get_objects_node()
      self.m02 = self.objects.add_object(idx, "M02")
     
      self.inlet = self.m02.add_variable(idx,"Inlet", False)
      self.outlet = self.m02.add_variable(idx,"Outlet", False)
      self.level = self.m02.add_variable(idx,"Level", 0)
      
      self.inlet.set_writable()
      self.outlet.set_writable()
      self.level.set_writable()

    
      self.server.start()
      print("Control Container Server running")

   def Stop_Server(self):
      self.server.stop()

# Create server & client
client = Control_Client()
server = Control_Server()

# Server is running for 5min
t_end = time.time() + 15 * 1

# Loop that forwards data from MPC container to M02 module
while time.time() < t_end:
  # read data from M02
  t_1 = time.time()
  M02_data = client.ReadFromM02()
  t_2 = time.time()
  t_d = (t_2 - t_1)*1000
  print(t_d)
  # write data to server
  server.level = M02_data

  # read new data from server
  new_data = [server.inlet.get_value(), server.outlet.get_value()]
  t_3 = time.time()
  print((t_3-t_1)*1000)
  print(new_data)

  # write data to M02
  try:
    client.WriteToM02(new_data)
    
  except:
    print("Error when writting to M02")

print("Control container stopped")
server.Stop_Server()

   