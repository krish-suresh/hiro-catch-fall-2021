import numpy as np
from math import cos, pi, sin
steps = np.linspace(0, pi/8, pi*2 )
r=0.2
for ang in steps:
    print([-cos(ang)*r, -0.4, 1+sin(ang)*r]), 
    print(ang)
