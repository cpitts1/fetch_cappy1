import numpy
import matplotlib.pyplot as plt
from copy import copy

def main():
    joint_pos = numpy.genfromtxt('example.txt')
    delta_t = 1.0/200 # measured in seconds
    joint_vel = difference(joint_pos,delta_t)
    joint_accel = difference(joint_vel,delta_t)

    t = numpy.arange(len(joint_pos)).reshape((-1, 1))*delta_t
    
    plt.subplot(3,1,1)
    plt.plot(t, joint_pos[:,0])
    plt.plot(t, joint_pos[:,1])
    plt.plot(t, joint_pos[:,2])

    plt.subplot(3,1,2)
    plt.plot(t, joint_vel[:,0])
    plt.plot(t, joint_vel[:,1])
    plt.plot(t, joint_vel[:,2])

    plt.subplot(3,1,3)
    plt.plot(t, joint_accel[:,0])
    plt.plot(t, joint_accel[:,1])
    plt.plot(t, joint_accel[:,2])
    plt.show()

# f'(x) ~ (-3*f(x) + f(x+h) - f(x+2*h)) / (2*h)
def three_point_forward(x0, x1, x2, h):
    return (-3.0*x0 + 4.0*x1 - x2)/(2.0*h)

# compute approximate derivatives thru central differencing of a
# this will operate on the ROWS of a
def difference(a, h):
    result = (a[2:,:] - a[:-2,:]) / (2*h)
    first = three_point_forward(a[0], a[1], a[2], h)
    last = -three_point_forward(a[-1], a[-2], a[-3], h)
    result = numpy.vstack( ( first,
                             result,
                             last ) )
    return result

main()
