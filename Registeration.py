import numpy as np

def reg_2d(line_set):
    #Construact the linear equtaion where Ax=B. A contain k abd B contain b
    A=line_set[:,0]
    ones=np.ones((np.size(A),1))
    np.hstack((A,ones))
    B=line_set[:,1]
    
    #Solve the crossing point using least square
    res=np.linalg.solve(A,B)
    y=res[0][0]
    x=res[1][0]
    return [x,y]