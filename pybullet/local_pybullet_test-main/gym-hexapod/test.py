
import pybullet as p
import time

p.connect(p.GRAPHICS_SERVER,"10.13.23.239")
#p.connect(p.GRAPHICS_SERVER)
print("started graphics server")
while p.isConnected():
  time.sleep(1./240.)
