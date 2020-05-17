# Calculates and draws the tangent lines for a circle and a line.
import argparse

def main():
    parser = argparse.ArgumentParser(description='Take input circle and point and find tangent.')
    parser.add_argument('--circle_radius', type=int, required=True, help='Radius of circle')
    parser.add_argument('--circle_center', type=int, nargs='+', required=True,
            help='Coordinates of circle center; x and y')
    parser.add_argument('--poi', type=int, nargs='+', required=True,
            help='Coordinates of point of interest; x and y')
    args = parser.parse_args()
    if len(args.poi) != 2 or len(args.circle_center) != 2:
        print('Invalid POI or circle center coordinates')
    circle_center = (float(args.circle_center[0]), float(args.circle_center[1]))
    circle_radius = float(args.circle_radius)
    poi = (float(args.poi[0]), float(args.poi[1]))

    tan_intersect_1, tan_intersect_2 = FindTangentIntersections(
            circle_center, circle_radius, poi)
    print(tan_intersect_1, tan_intersect_2)

# Test case 1:
#  circle_center (5, 4)
#  circle_radius 4
#  poi (11, 13)
#  Expect {m1, m2} \approx { 0.690025, 4.709975 }
#  See http://desmos.com/calculator/msceuvx6fh
# Test case 2:
#  circle_center (200, 300)
#  circle_radius 50
#  poi (210, 100)
#  Expect {m1, m2} \approx { 4.873067, -3.206400 }
#  See http://desmos.com/calculator/10wmhadozg

def FindSlopesTangentToCircle(circle_center, circle_radius, poi):
    px, py = poi
    cx, cy = circle_center
    R = circle_radius
    # The equation of a line with slope `m` going through `poi` is:
    #   y - py = m ( x - px ).
    # The equation of the circle in question is:
    #   ( y - cy )^2 + ( x - cx )^2 = R^2
    # Thus, the intersection of the two curves is given by the equation:
    #   ( m*x - m*px - py - cy )^2 + ( x - cx )^2 = R^2

    # Expanding and solving for x (algebra left as an exercise to the reader)
    # results in solving a quadratic, and depending on the value of m, the
    # quadratic solution will have zero, one, or two real valued solutions.

    # The fully expanded quadratic is:
    #   (m^2 + 1)x^2 + (2m*py - 2m^2*px - 2m * cy - 2cx)x
    #     + cx^2 - R^2 + (py - m*px - cy)^2 = 0
    # Its roots represent the x values of the intersection(s).

    # We are interested in the tangent lines, which are those lines for which
    # the slope results in exactly one real valued solution for x.
    # (Two real valued solutions indicates the line intersects the circle twice,
    # and zero real valued solutions would indicate that there are no
    # intersections between the circle and a line with that slope.

    # We are interested in the tangent line, so we make use of the discriminant
    # and observe at which values of m (slope) the discriminant is equal to
    # zero. These slopes will be the ones that have exactly one real valued
    # solution for x, and thus the slopes which form tangent lines.

    # The discriminant is equal to zero exactly when:
    #  (2m*py - 2px*m^2 - 2m*cy - 2cx) = (4m^2 + 4)(cx^2 - R^2 + (py - m*px - cy)^2)

    # We can fully expand and simplify this equation, representing it as a
    # quadratic in terms of the slope `m`; A*m**2 + B*m + C = 0, where:
    A = (8*cx*px - 4*px*px - 4*cx*cx + 4*R*R)
    B = (8*cx*cy - 8*cx*py - 8*cy*px + 8*px*py)
    C = (8*cy*py - 4*cy*cy - 4*py*py + 4*R*R)
    # Thus, values of m at which the line is tangent to the circle are:
    m1 = ( -B + (B**2 - 4*A*C)**(0.5) ) / (2*A)
    m2 = ( -B - (B**2 - 4*A*C)**(0.5) ) / (2*A)
    print('Determined slopes should be {} and {}'.format(m1, m2))
    return m1, m2


def FindTangentIntersections(circle_center, circle_radius, poi):
    """ Finds the two intersections (if they exist) between the the circle and
    the tangent lines going through poi"""
    m1, m2 = FindSlopesTangentToCircle(circle_center, circle_radius, poi)
    # calculate intersections based on slopes
    def IntersectionOfCircleAndLine(
            circle_center, circle_radius, point_on_line, slope):
        R = circle_radius
        cx, cy = circle_center
        px, py = point_on_line
        m = slope
        # Once again, we get
        #   ( m*x - m*px - py - cy )**2 + ( x - cx )**2 = R**2
        # Which once again reduces to:
        #   (m^2 + 1)x^2 + (2m*py - 2m^2*px - 2m * cy - 2cx)x
        #     + cx^2 - R^2 + (py - m*px - cy)^2 = 0
        # Only this time, we know the value of `m` (the slope)!
        A = (m*m + 1)
        B = (2*m*py - 2*m*m*px - 2*m*cy - 2*cx)
        C = (cx*cx - R*R + (py - m*px - cy)**2)
        x1 = ( -B + (B**2 - 4*A*C)**(0.5) ) / (2*A)
        x2 = ( -B - (B**2 - 4*A*C)**(0.5) ) / (2*A)
        print('Intersection of line and circle at x = {{{}, {}}}'.format(x1, x2))
        y1 = m * ( x1 - px ) + py
        return x1, y1
    tangent_1 = IntersectionOfCircleAndLine(circle_center, circle_radius, poi, m1)
    tangent_2 = IntersectionOfCircleAndLine(circle_center, circle_radius, poi, m2)
    return (tangent_1, tangent_2)

main()