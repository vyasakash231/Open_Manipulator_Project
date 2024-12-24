import numpy as np
import matplotlib.pyplot as Plot

lam = np.linspace(0,1,10)

start_pt = np.array([2,3])
end_pt = np.array([10,7])

X = np.array([])
Y = np.array([])

for _ in range(0,len(lam)):
    X = np.concatenate((X,np.array([lam[_] * end_pt[0] + (1 - lam[_]) * start_pt[0]])))
    Y = np.concatenate((Y,np.array([lam[_] * end_pt[1] + (1 - lam[_]) * start_pt[1]])))

fig = Plot.figure()
Plot.plot(X,Y, 'r-')
Plot.xlabel('X')
Plot.ylabel('Y')
Plot.legend(['$End Effector Point$'])
Plot.xlim([0,12])
Plot.ylim([0,12])
Plot.grid()

Plot.show()