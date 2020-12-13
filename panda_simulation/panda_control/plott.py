import rospy
import numpy as np
from math import sin, cos
from sensor_msgs.msg import JointState
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def callback(data):
	scope.update(data.position)

class Scope(object):
    def __init__(self, ax, maxt=2, dt=0.001):
        self.ax = ax
        self.dt = dt
        self.maxt = maxt
        self.tdata = [0]
        self.ydata = [0]
        self.ydata2 = [0]
        self.ydata3 = [0]
        self.ydata4 = [0]
        self.ydata5 = [0]
        self.ydata6 = [0]
        self.ydata7 = [0]
        self.line = Line2D(self.tdata, self.ydata)
        self.line2 = Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.add_line(self.line2)
        #self.ax.set_ylim(-.1, 1.1)
        #self.ax.set_xlim(0, self.maxt)

    def update(self, y):

        t = self.tdata[-1] + self.dt
        self.tdata.append(t)
        self.ydata.append(y[0] )
        self.ydata2.append(y[1] )
        self.ydata3.append(y[2] )
        self.ydata4.append(y[3] )
        self.ydata5.append(y[4] )
        self.ydata6.append(y[5] )
        self.ydata7.append(y[6] )
     
        if len(self.tdata)>3000:
                self.tdata=[0]
                self.ydata = [0]
                self.ydata2 = [0]  
                self.ydata3 = [0]
                self.ydata4 = [0]  
                self.ydata5 = [0]
                self.ydata6 = [0]  
                self.ydata7 = [0]
                      
#        self.line.set_data(self.tdata, self.ydata)
#        self.line2.set_data(self.tdata, self.ydata2)
        #return (self.line,self.line2)



fig, (ax1, ax2, ax3, ax4, ax5, ax6, ax7) = plt.subplots(1,7)
scope = Scope(ax1)

rospy.init_node('inv_dyn', anonymous=True)
rospy.Subscriber("/robot1/joint_states", JointState, callback)
rate = rospy.Rate(100) # 1000hz
# pass a generator in "emitter" to produce data for the update func
#ani = animation.FuncAnimation(fig, scope.update, callback, interval=10,  blit=True)
#plt.ion()
while not rospy.is_shutdown():
	ax1.plot(scope.ydata)
	ax2.plot(scope.ydata2)
	ax3.plot(scope.ydata3)
	ax4.plot(scope.ydata4)
	ax5.plot(scope.ydata5)
	ax6.plot(scope.ydata6)
	ax7.plot(scope.ydata7)

	#plt.show()
	plt.pause(0.3)
plt.ioff()    
plt.show()

