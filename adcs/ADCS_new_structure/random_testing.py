import numpy as np

x = np.array([-1,-2.1,1])
max_x = np.array([2,2,2])

if (np.any(np.absolute(x) > max_x)):
    temp_x = max_x * np.sign(x) * (np.absolute(x) > max_x)
    x = temp_x + x*(np.absolute(x) < max_x)
    
print(x)