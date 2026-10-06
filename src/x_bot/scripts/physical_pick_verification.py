"""Read-only simulation acceptance checks; never supplies grasp targets or motion poses."""
import math

CLASS_OBJECTS = {3: 'book', 2: 'coffee_mug', 9: 'coke_can', 4: 'water_bottle', 6: 'shoe'}

def lifted(before, after, name, min_lift=0.06):
    obj = after['objects'][name]['position']
    initial = before['objects'][name]['position']
    hand = after['objects']['fr3_hand']['position']
    return obj[2] - initial[2] >= min_lift and math.dist(obj, hand) < 0.28

def retained(lift, placed, name, tolerance=0.08):
    def relative(state):
        obj = state['objects'][name]['position']
        hand = state['objects']['fr3_hand']['position']
        hand_state = state['objects']['fr3_hand']
        w, x, y, z = hand_state.get('orientation_wxyz', [1., 0., 0., 0.])
        # Express the object offset in hand coordinates (inverse rotation).
        dx, dy, dz = (obj[i]-hand[i] for i in range(3))
        return [(1-2*(y*y+z*z))*dx + 2*(x*y+w*z)*dy + 2*(x*z-w*y)*dz,
                2*(x*y-w*z)*dx + (1-2*(x*x+z*z))*dy + 2*(y*z+w*x)*dz,
                2*(x*z+w*y)*dx + 2*(y*z-w*x)*dy + (1-2*(x*x+y*y))*dz]
    return math.dist(relative(lift), relative(placed)) < tolerance

def in_bin(state, name):
    point = state['objects'][name].get('center', state['objects'][name]['position'])
    bounds = state.get('place_bounds')
    if not bounds:
        return False
    lo, hi = bounds
    return all(lo[i] <= point[i] <= hi[i] for i in range(3))
