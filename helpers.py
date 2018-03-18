import numpy as np

def euclid_distance(p1,p2):
    return np.sqrt(np.power(p1[0]-p2[0],2) + np.power(p1[1]-p2[1],2))

def intersect(line1,line2):
    """
    checks if two lines collides each other.
    :input:
        line1 : [[x1,y1],[x2,y2]]
        line2 : as line 1
    :returns:
        a boolean
    """
    vec1 = [line1[1][0]-line1[0][0],line1[1][1]-line1[0][1]]
    vec2 = [line2[1][0]-line2[0][0],line2[1][1]-line2[0][1]]
    cross = vec1[0]*vec2[1] - vec1[1]*vec2[0]
    if (cross == 0):
        return False
    p_minus_q = [line1[0][0]-line2[0][0],line1[0][1] - line2[0][1]]
    q_minus_p = [line2[0][0]-line1[0][0],line2[0][1] - line1[0][1]]
    cross1 = q_minus_p[0]*vec2[1] - q_minus_p[1]*vec2[0]
    cross2 = q_minus_p[0]*vec1[1] - q_minus_p[1]*vec1[0]
    t = cross1/cross
    u = cross2/cross
    if (0 < t < 1 and 0 < u < 1):
        return True
    return False


