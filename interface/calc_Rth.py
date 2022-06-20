from scipy.constants import k, pi
import numpy as np


f_0 = float(input('What is the resonance frequency (f_0) ? [Hz] '))
k_b = float(k)
T = float(input('What is the temperature in the room? [K]'))
k_0 = float(input('What is the Spring constant of the cantilever ?(Auto Tune) [N/m]'))
Q = float(input('What is the Q-factor? (Sweeper)'))
A = float(input('What is the Oscillation Amplitude? [V]'))
Amp_InvOLS = float(input('What is the Amp InvOLS? [nm/V]'))

Amp = A * np.sqrt(2)*Amp_InvOLS*1e-9

Rth = (f_0*k_b*T)/(pi*k_0*Q*Amp**2)
print('The thermal noise Rth is: [Hz]',Rth)