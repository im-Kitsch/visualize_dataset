import numpy as np
import time

begin_time = time.time()

for i in range(10000):
    a = np.random.random((3, 4))
    b = np.random.random((4, 5))
    tt = np.dot(a, b)


print time.time() - begin_time
