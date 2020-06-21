# Calculates and draws the tangent lines for a circle and a line.
from PIL import Image, ImageDraw
import argparse
import math
_X_PIXEL_COUNT = 600
_Y_PIXEL_COUNT = 458
_Y_OFFSET = _Y_PIXEL_COUNT
_LIGHT_GREY = '#d4d4d4'
_INTERSECT_1 = 'INTERSECT_1'
_INTERSECT_2 = 'INTERSECT_2'

# Returns (x0, y0, x1, y1) where (x0, y0) is the upper left hand corner of the
# bounding box and (x1, y1) is the bottom right hand corner of the bounding
# box of the circle defined by the given center and radius.
def BoundingBoxOfCircle(center, radius):
    x0 = center[0] - radius
    y0 = center[1] - radius
    x1 = center[0] + radius
    y1 = center[1] + radius
    return (x0, y0, x1, y1)

def DrawArcInBox(draw, bounding_box, fill=None):
    draw.arc((bounding_box[0], _Y_OFFSET - bounding_box[3],
        bounding_box[2], _Y_OFFSET - bounding_box[1]),
        start=0, end=360, fill=fill)

def MarkPoint(draw, point):
    # to draw points, we will draw a tiny circle centered at the point
    DrawArcInBox(draw, BoundingBoxOfCircle(point, 2), fill='blue')

def DrawSegment(draw, p1, p2):
    draw.line(((p1[0], _Y_OFFSET - p1[1]),
              (p2[0], _Y_OFFSET - p2[1])))


def InterpolateArcBetween(p1, p2, center, is_left_circle, radius, speed):
    p1_theta = math.atan2(p1[1] - center[1], p1[0] - center[0])
    p2_theta = math.atan2(p2[1] - center[1], p2[0] - center[0])
    arc_length_step_size = speed
    d_theta = arc_length_step_size / radius
    theta = p1_theta
    points = []
    print('p1_theta: {}, p2_theta: {}, d_theta: {}'.format(p1_theta, p2_theta, d_theta))
    # make sure this will terminate
    if is_left_circle:
        while theta < p2_theta:
            theta += d_theta
            points.append((radius * math.cos(theta) + center[0],
                           radius * math.sin(theta) + center[1]))
    else:
        while theta > p2_theta:
            theta -= d_theta
            points.append((radius * math.cos(theta) + center[0],
                           radius * math.sin(theta) + center[1]))
    return points


def InterpolateLineSegmentBetween(p1, p2, speed):
    delta_y = p2[1] - p1[1]
    delta_x = p2[0] - p1[0]
    distance = math.hypot(delta_x, delta_y)
    dx = speed * math.cos(math.atan2(delta_y, delta_x))
    dy = speed * math.sin(math.atan2(delta_y, delta_x))
    steps_needed = int(distance / speed)
    points = []
    for i in range(steps_needed):
        points.append((p1[0] + i * dx, p1[1] + i * dy))
    return points

def Distance(p1, p2):
    return math.hypot(p2[1] - p1[1], p2[0] - p1[0])


def DetermineCircle(player_location, player_orientation, poi, circle_radius):
    # the angle to the center of the circle on the left of the player
    angle_to_lcc = player_orientation + math.pi / 2
    # the angle to the center of the circle on the right of the player
    angle_to_rcc = player_orientation - math.pi / 2
    left_circle_center = (player_location[0] + circle_radius * math.cos(angle_to_lcc),
                          player_location[1] + circle_radius * math.sin(angle_to_lcc))
    right_circle_center = (player_location[0] + circle_radius * math.cos(angle_to_rcc),
                           player_location[1] + circle_radius * math.sin(angle_to_rcc))
    distance_to_left = Distance(left_circle_center, poi)
    distance_to_right = Distance(right_circle_center, poi)
    if distance_to_left < distance_to_right:
        return left_circle_center, True
    return right_circle_center, False


def DetermineFirstIntersectionPointEncounteredMovingCounterClockwise(
        player_location, tan_intersect_1, tan_intersect_2, circle_center):
    theta_1 = math.atan2(tan_intersect_1[1] - circle_center[1],
            tan_intersect_1[0] - circle_center[0])
    theta_2 = math.atan2(tan_intersect_2[1] - circle_center[1],
            tan_intersect_2[0] - circle_center[0])
    theta_player = math.atan2(player_location[1] - circle_center[1],
            player_location[0] - circle_center[0])
    if ((theta_player < theta_1 and theta_player < theta_2) or
            (theta_player > theta_1 and theta_player > theta_2)):
        return _INTERSECT_1 if theta_1 < theta_2 else _INTERSECT_2
    # theta_player is located in between the two tanget line intersections
        return _INTERSECT_1 if theta_1 > theta_2 else _INTERSECT_2


def SelectAppropriateWaypoint(player_location, tan_intersect_1, tan_intersect_2, circle_center, is_left_circle):
    first_intersection_ccw = DetermineFirstIntersectionPointEncounteredMovingCounterClockwise(
            player_location, tan_intersect_1, tan_intersect_2, circle_center)
    # print('player_location {}, tan_intersect_1 {}, tan_intersect_2 {}, circle_center {}, first_intersection_ccw {}'.format(
    #    player_location, tan_intersect_1, tan_intersect_2, circle_center, first_intersection_ccw))
    if is_left_circle:
        return tan_intersect_1 if first_intersection_ccw == _INTERSECT_1 else tan_intersect_2
    # if it is a right hand circle, the player will be travelling
    # clockwise along it, which means we can simply invert which
    # intersection point is closer.
    print('Using {}'.format(
        'tan_intersect_2' if first_intersection_ccw == _INTERSECT_1 else 'Using tan_intersect_1'))
    return tan_intersect_2 if first_intersection_ccw == _INTERSECT_1 else tan_intersect_1


def main():
    parser = argparse.ArgumentParser(description='Take input circle and point and find tangent.')
    parser.add_argument('--image_directory', type=str, required=False,
            help='Path to directory where we will write output images.')
    parser.add_argument('--turning_radius', type=int, required=True, help='Radius of circle')
    parser.add_argument('--player_location', type=int, nargs='+', required=True,
            help='Coordinates of circle center; x and y')
    parser.add_argument('--poi', type=int, nargs='+', required=True,
            help='Coordinates of point of interest; x and y')
    args = parser.parse_args()
    if len(args.poi) != 2 or len(args.player_location) != 2:
        print('Invalid POI or player location coordinates')
    path = args.image_directory
    poi = (float(args.poi[0]), float(args.poi[1]))
    player_location = (float(args.player_location[0]), float(args.player_location[1]))
    circle_radius = float(args.turning_radius)
    player_orientation = -math.pi / 4
    circle_center, is_left_circle = DetermineCircle(player_location, player_orientation, poi, circle_radius)
    print('Will use the {} circle.'.format('left' if is_left_circle else 'right'))

    canvas = Image.new('RGB', (_X_PIXEL_COUNT, _Y_PIXEL_COUNT), color=_LIGHT_GREY)
    draw = ImageDraw.Draw(canvas)
    MarkPoint(draw, poi)
    MarkPoint(draw, player_location)
    canvas.save('{}/1_player_and_destination.png'.format(path))
    DrawArcInBox(draw, BoundingBoxOfCircle(circle_center , circle_radius))
    canvas.save('{}/2_circle_tangent.png'.format(path))

    tan_intersect_1, tan_intersect_2 = FindTangentIntersections(
            circle_center, circle_radius, poi)
    print('Found {} and {} as tangent intersections.'.format(tan_intersect_1, tan_intersect_2))

    DrawSegment(draw, tan_intersect_1, poi)
    DrawSegment(draw, tan_intersect_2, poi)
    canvas.save('{}/3_tangent_intersections.png'.format(path))


    waypoint = SelectAppropriateWaypoint(player_location, tan_intersect_1, tan_intersect_2, circle_center, is_left_circle)
    print('Picked {} as waypoint.'.format(waypoint))
    # how fast the object can travel
    speed = 7.0
    arc_points = InterpolateArcBetween(player_location, waypoint,
            circle_center, is_left_circle, circle_radius, speed)
    line_segment_points = InterpolateLineSegmentBetween(waypoint, poi, speed)
    print('{} arc points, {} line segment points.'.format(
        len(arc_points), len(line_segment_points)))
    for p in arc_points + line_segment_points:
        MarkPoint(draw, p)

    canvas.save('{}/4_path_to_destination.png'.format(path))


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
        return x1.real, y1.real
    tangent_1 = IntersectionOfCircleAndLine(circle_center, circle_radius, poi, m1)
    tangent_2 = IntersectionOfCircleAndLine(circle_center, circle_radius, poi, m2)
    return (tangent_1, tangent_2)

main()
