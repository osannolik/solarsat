import numpy as np

### Limits ###

Vbg = 1.23

to_limits = lambda R4,R5,R6: (Vbg * (R4+R5+R6)/(R5+R6), Vbg * (R4+R5+R6)/R6)

R4 = 10.0

to_r5_r6 = lambda Vuvp,Veoc: np.linalg.solve(np.array([[Vuvp-Vbg, Vuvp-Vbg], [-Vbg, Veoc-Vbg]]), np.array([R4*Vbg, R4*Vbg]))

to_r5_r6(3.3, 4.2)
# array([1.27329193, 4.66873706])

# Pick from E12-series
R5 = 1.2
R6 = 4.7
(uvp, eoc) = to_limits(R4, R5, R6) 
# (3.314745762711864, 4.161063829787234)

print("R4 = {} M".format(R4))
print("R5 = {} M".format(R5))
print("R6 = {} M".format(R6))
print("Vuvp = {} V".format(uvp))
print("Veoc = {} V".format(eoc))


### MPPT ###

R1 = 10.0

# For N SM141K10L in series
N = 2.0
Voc = N * 6.91
Vmppt = N * 5.58
mppt_ratio = Vmppt / Voc
Vuvp_min = max(2.2, uvp)
R2_R3_sum = R1 * Vuvp_min/(Voc-Vuvp_min) # due to mppt pin limited to Vuvp minimum
R3 = mppt_ratio * R2_R3_sum
R2 = R2_R3_sum - R3

#R2 = 0.6073195345261375 M
#R3 = 2.548002257635976 M

# Pick from E48-series
R2 = 0.59
R3 = 2.49

print("R1 = {} M".format(R1))
print("R2 = {} M".format(R2))
print("R3 = {} M".format(R3))
print("set ratio {}".format(R3 / (R2+R3)))

Vmpp = Voc * (R2+R3) / (R1+R2+R3)
Vmpp_set = Voc * R3 / (R1+R2+R3)

print("Vmpp = {} V at Voc = {}".format(Vmpp,Voc))
print("Vmpp_set = {} V at Voc = {}".format(Vmpp_set,Voc))
