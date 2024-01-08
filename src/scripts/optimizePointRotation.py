import numpy as np

def f_optimizePointOrientation(self, T_init):

    
    R1 = T_init[:3, :3]  # Initial rotation matrix
    P1 = T_init[:3, 3]   # Initial position vector

    # Optimize position using fmin
    P1_optimized = fmin(func=self.objectiveFunPosition, x0=P1, args=(R1, self.CalipenCalibrationData_list), xtol=1e-4, ftol=1e-4,  disp=True)
    #P1_optimized = minimize(fun=self.objectiveFunPosition, x0=P1, args=(R1), method="Nelder-Mead", tol=1e-1)

    print("optimization finished")
    print(P1_optimized)
    T = np.eye(4)
    T[:3, 3] = P1_optimized

    return T