 
import numpy as np
from matplotlib import pyplot as plt
from pdb import set_trace
x = np.linspace(-2,  2, 300 )
 

n = 1; s=0; 

cs = [0.1,0.3,0.5]

#plt.ylim([-6,6])

"""
for c in [0.2]:
	for w in [1, 3, 6]:
		for r in [0.3]:
			f = w * (   np.power(-1, n) * np.exp(  -np.power(x-s, 2) / (2*np.power(c, 2))  ) + r* np.power(x-s, 4)  )
		
			plt.plot(x, f )

plt.legend(loc="upper left")
plt.show()
"""
w=50; n=1; s=0; c=0.2; r=5;
#f_p = w * ( np.power(-1, n) * np.exp(  -np.power(x-s, 2) / (2*np.power(c, 2))  ) + r* np.power(x-s, 4) )
f_p = w * (  r* np.power(x-s, 4) )
w=40; n=1; s=0; c=0.2; r=5;
#f_o = w * ( np.power(-1, n) * np.exp(  -np.power(x-s, 2) / (2*np.power(c, 2))  ) + r* np.power(x-s, 4) )
f_o = w * (  r* np.power(x-s, 4) )		
plt.plot(x, f_p, label="pos")
plt.plot(x, f_o, label="ori")
#plt.plot(x, f_p+f_o, label="pos+ori")
plt.legend(loc="upper left")
plt.show()


"""
#w p =50, w o = 40, w v = 0.1, w a = 1, w j = 2, w e = 0.1, w c = 2
"""
