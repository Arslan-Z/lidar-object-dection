import numpy as np
import matplotlib.pyplot as plt

def LB_line(alpha):
    theta = np.linspace(0,2*np.pi,num=500)   

    x1 =   (alpha * np.sqrt(2) * np.cos(theta) / (np.sin(theta)**2+1) ).tolist()  
    y1 =   (alpha * np.sqrt(2) * np.cos(theta) * np.sin(theta)/(np.sin(theta)**2+1)  ).tolist() 

    x2  =  ( ( alpha - 0.4 )  * np.sqrt(2) * np.cos(theta) / (np.sin(theta)**2+1) ).tolist() 
    y2  =  ( ( alpha - 0.4 )  * np.sqrt(2) * np.cos(theta) * np.sin(theta)/(np.sin(theta)**2+1)  ).tolist() 

    x3  =  ( (  alpha - 0.8 )  * np.sqrt(2) * np.cos(theta) / (np.sin(theta)**2+1) ).tolist() 
    y3  =  ( ( alpha - 0.8 )  * np.sqrt(2) * np.cos(theta) * np.sin(theta)/(np.sin(theta)**2+1)   ).tolist() 

    x4  =  ( ( alpha - 1.2 )  * np.sqrt(2) * np.cos(theta) / (np.sin(theta)**2+1) ).tolist() 
    y4  =  ( ( alpha - 1.2 )  * np.sqrt(2) * np.cos(theta) * np.sin(theta)/(np.sin(theta)**2+1)   ).tolist() 
    
    
    return x1 + x2 + x3 + x4,  y1 + y2 + y3 + y4

def rotate(x_, y_, theta):
    x1 = []
    y1 = []
    for idx, x in enumerate(x_): 
        y = y_[idx]
        x1.append( x * np.cos(theta) - y * np.sin(theta) ) 
        y1.append( x * np.sin(theta) + y * np.cos(theta) )

    return x1, y1


def spectral(x_, y_, num ):
    theta = [0.3 * x for x in range(num)]
    print(theta)
    x_all = []
    y_all = []
    # x_all.append(x_)
    # y_all.append(y_)
    for t in theta:
        x1 = []
        y1 = []
        x1, y1 = rotate(x_, y_, t)
        x_all.append(x1)
        y_all.append(y1)

    return x_all, y_all

# plt.title(r"$\rho^{2}=a^{2}\cos 2\theta\quad a=1$")
x1, y1 = LB_line(8) 
plt.subplot(221)
plt.title(r"$\rho^{2}=a^{2}\cos 2\theta\quad a=1$")
plt.scatter(x1, y1, s = 1)
 
plt.subplot(222)
x1, y1 = LB_line(8)
xa, ya = spectral(x1, y1, 10) 
plt.scatter(xa, ya, s = 1)

plt.subplot(223)
x1, y1 = LB_line(8)
xa, ya = spectral(x1, y1, 20) 
plt.scatter(xa, ya, s = 1)

plt.subplot(224)
x1, y1 = LB_line(8)
xa, ya = spectral(x1, y1, 100) 
plt.scatter(xa, ya, s = 1)


# plt.grid()
plt.show()
 