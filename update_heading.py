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


def ExtendArcFromPoint(p1, center, radius, d_theta):
    p1_theta = math.atan2(p1[1] - center[1], p1[0] - center[0])
    theta = p1_theta + d_theta
    return (radius * math.cos(theta) + center[0],
            radius * math.sin(theta) + center[1])


def DrawHeadingAtLocation(draw, p1, normal):
    length = 40
    p2 = (p1[0] + length * normal[0],
            p1[1] + length * normal[1])
    DrawSegment(draw, p1, p2)


def HeadingFromTangentPoint(circle_center, p):
    if circle_center[1] == p[1]:
        # if the tangent point is exactly on the edge, then the tangent line is
        # perfectly vertical and thus the slope would be undefined. We can
        # special case this by returning the appropriate vector for the case.
        print('special case! p = {}, c = {}'.format(p, circle_center))
        return (0,1) if p[0] < circle_center[0] else (0,-1)
    c = circle_center
    # inverse reciprocal of the slope between the lines
    tangent_line_slope = (c[0] - p[0]) / (p[1] - c[1])
    # the function giving the y coordinate of the tangent line
    def y(x):
        return tangent_line_slope * (x - p[0]) + p[1]
    orientation_vec = (1, y(p[0] + 1) - p[1])
    magnitude = math.sqrt(orientation_vec[0] * orientation_vec[0] +
            orientation_vec[1] * orientation_vec[1])
    return (orientation_vec[0] / magnitude,
            orientation_vec[1] / magnitude)


def main():
    parser = argparse.ArgumentParser(description='Updates heading')
    parser.add_argument('--image_directory', type=str, required=False,
            help='Path to directory where we will write output images.')
    args = parser.parse_args()
    path = args.image_directory
    circle_center = (100, 100)
    circle_radius = 60
    player_orientation = -math.pi / 4
    player_heading = [math.sqrt(2)/2, -math.sqrt(2)/2]
    player_location = [circle_center[0] + circle_radius * math.sqrt(2)/2,
            circle_center[1] + circle_radius * math.sqrt(2)/2]

    canvas = Image.new('RGB', (_X_PIXEL_COUNT, _Y_PIXEL_COUNT), color=_LIGHT_GREY)
    draw = ImageDraw.Draw(canvas)
    DrawArcInBox(draw, BoundingBoxOfCircle(circle_center, circle_radius))
    MarkPoint(draw, player_location)
    DrawHeadingAtLocation(draw, player_location, player_heading)
    canvas.save('{}/heading_rotation_1.png'.format(path))
    d_theta = -0.05 * math.pi
    # move along the circle
    def MoveAlongCircleAndDraw(draw, player_location, player_heading,
            circle_center, circle_radius, d_theta):
        new_player_location = ExtendArcFromPoint(
                player_location, circle_center, circle_radius, d_theta)
        new_heading = HeadingFromTangentPoint(
                circle_center, new_player_location)
        if new_player_location[1] < circle_center[1]:
            new_heading = (-1 * new_heading[0], -1 * new_heading[1])
        player_location[:] = new_player_location[:]
        player_heading[:] = new_heading[:]
        MarkPoint(draw, new_player_location)
        DrawHeadingAtLocation(draw, player_location, player_heading)

    for i in range(2, 50):
        MoveAlongCircleAndDraw(draw, player_location, player_heading,
                circle_center, circle_radius, d_theta)
        canvas.save('{}/heading_rotation_{}.png'.format(path, i))


main()
