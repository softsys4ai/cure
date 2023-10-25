import os
import pickle
from goals_husky import *

def rns(currPose, target):
    RNS = 0
    with open('log/rns', 'ab+') as fp:
        if currPose >= target:
            RNS += 1
            pickle.dump(RNS,fp)
        else:
            RNS = 0
            pickle.dump(RNS,fp)

def CalcRns():
    data = []
    with open('log/rns', 'rb') as fr:
        try:
            while True:
                data.append(pickle.load(fr))
                RNS = sum(data)
                RNS = float(RNS)
                robustness = RNS/narrow_spaces
                with open('log/robustness_narrow_space.txt', 'w') as f:
                    f.write(str(robustness))
                    f.close()  
        except EOFError:
            pass

def ResetRNS():
    os.remove("log/rns")
