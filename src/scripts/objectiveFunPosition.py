import numpy as np

def objectiveFunPosition(self, P, R, T_data):
    """
    Optimization for position
    """
    T0A = T_data.copy()
    Tx = np.eye(4)
    Tx[:3, :3] = R
    Tx[:3, 3] = P

    num_of_mesurs = len(T0A)  # holds the number of measurements
    f = 0  # optimization function value initialized to 0
    print(num_of_mesurs)

    for i in range(0, num_of_mesurs):
        T0o1_old = T0A[i] @ Tx
        for j in range(0, num_of_mesurs):
            T0o1 = T0A[j] @ Tx
            p = T0o1[:3, 3] - T0o1_old[:3, 3]
            #print(p)
            f += np.linalg.norm(p)  # Euclidean distance
    print(f)
    return f