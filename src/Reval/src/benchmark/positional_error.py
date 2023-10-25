import math

def EuclideanDistance(actual_x, actual_y, robot_x, robot_y):    
    euc_distance = math.sqrt((actual_x - robot_x) ** 2 + (actual_y - robot_y) ** 2)
    return  euc_distance