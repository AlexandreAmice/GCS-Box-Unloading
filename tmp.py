import numpy as np
import matplotlib.pyplot as plt

smallest = 1e-8
delta = np.linspace(smallest,1,100)
p = np.linspace(smallest,1, 100)
tau = 0.5

D, P = np.meshgrid(delta, p)
Z = np.ceil(-2*np.log(D)/(tau * tau * p) + 0.5)

thresh = (Z > 0)
plt.imshow(thresh, cmap='gray')
plt.show()
