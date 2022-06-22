import numpy as np

def reg_2d(line_set):
    #Construact the linear equtaion where Ax=B. A contain k abd B contain b
    line_set=np.array(line_set)
    x=line_set[:,0]
    A=np.vstack([x, np.ones(len(x))]).T
    B=line_set[:,1]
    
    #Solve the crossing point using least square
    res=np.linalg.lstsq(A,B,rcond=None)
    y=res[0][0]
    x=res[1][0]
    return [x,y]