def onSegment(p, q, r):
    if (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
        return True
    return False

def orientation(p, q, r):

    val = (q[1] - p[1]) * (r[0] - q[0]) -  (q[0] - p[0]) * (r[1] - q[1]) 
  
    if (val == 0):
        return 0#;  // colinear
    if(val > 0): # clockwise
        return 1
    return 2# counter clockwise
    

  
def doIntersect(p1, q1, p2, q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2) 
    o3 = orientation(p2, q2, p1) 
    o4 = orientation(p2, q2, q1) 
  
    if (o1 != o2 and o3 != o4):
        return True 
  
    if (o1 == 0 and onSegment(p1, p2, q1)):
        return True 
  
    if (o2 == 0 and onSegment(p1, q2, q1)):
        return True 
  
    if (o3 == 0 and onSegment(p2, p1, q2)):
        return True
  
    if (o4 == 0 and onSegment(p2, q1, q2)):
        return True 
  
    return False #; // Doesn't fall in any of the above cases 
  
def isInside(polygon, n, p): 
    if (n < 3):
        return False
    extreme = (65535, p[1]) 
    count = 0
    i = 0
    first = True
    while (i != 0 or first == True):
        first = False
        _next = (i+1)%n
        if (doIntersect(polygon[i], polygon[_next], p, extreme)):
            if (orientation(polygon[i], p, polygon[_next]) == 0):
               return onSegment(polygon[i], p, polygon[_next])
            count+=1
        i = _next
  
    return count%2 == 1

if __name__ == "__main__":
    polygon1 = [(0,0),(10,0),(10,10),(0,10)]
    n = 4
    p = (20, 20) 
    print(p, isInside(polygon1, n, p))
  
    p = (5, 5)
    print(p, isInside(polygon1, n, p))


