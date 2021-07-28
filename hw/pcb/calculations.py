import numpy as np

to_limits = lambda R4,R5,R6: (1.23 * (R4+R5+R6)/(R5+R6), 1.23 * (R4+R5+R6)/R6)

Vbg = 1.23
R4 = 10

to_r5_r6 = lambda Vuvp,Veoc: np.linalg.solve(np.array([[Vuvp-Vbg, Vuvp-Vbg], [-Vbg, Veoc-Vbg]]), np.array([R4*Vbg, R4*Vbg]))

to_r5_r6(3.3, 4.2)
# array([1.27329193, 4.66873706])

to_limits(R4, 1.2, 4.7) # E12-series
# (3.314745762711864, 4.161063829787234)
