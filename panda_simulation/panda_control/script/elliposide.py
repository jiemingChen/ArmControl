 
import numpy as np
from matplotlib import pyplot as plt
from pdb import set_trace
from mpl_toolkits import mplot3d

x = np.linspace(-2,  7, 300 )
y = np.linspace(-2,  7, 300 )

f_p  =  np.power( (1 - np.power(x-2,2) / 4)*16, 0.5)+5
f_p2 = -np.power( (1 - np.power(x-2,2) / 4)*16, 0.5)+5  		

#ax = plt.axes(projection='3d')

plt.plot(x, f_p);
plt.plot(x, f_p2);
 
plt.show()



