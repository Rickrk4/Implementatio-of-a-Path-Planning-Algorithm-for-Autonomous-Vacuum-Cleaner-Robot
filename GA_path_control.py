from path_control import PathControl

def load_array(file, delimiter=','):
    text_file = open(file, "r")
    lines = text_file.read().replace('\n', '')[2:-2].split(']' + delimiter + ' [')
    l = list(map( lambda str:  list(map(lambda x : int(x), str.split(delimiter))), lines))
    return l


def array_to_str(a):
    return ''.join(str(x) for x in a)


def compress_array(a, px, py, theta, invert_y_axis=False):
    _y = -1 if invert_y_axis else 1

    dirs = {
        "01": 90,
        "11": 45,
        "10": 0,
        "1-1": 315,
        "0-1": 270,
        "-1-1": 225,
        "-10": 180,
        "-11": 135
    }

    current_theta = theta
    current_position = [px, py]
    r = []
    p = a[0]
    r.append(('rotate', [dirs[array_to_str(p)]]))
    c = 1

    for x in a[1:]:
        if (p == x):
            c = c + 1
        else:
            current_position = [current_position[0] + p[0] * c * _y, current_position[1] + p[1] * c]
            r.append(('go_to',
                      [round(0.5 - current_position[0] / 10.0 - 0.05, 2), round(current_position[1] / 10.0 + 0.05, 2)]))
            theta = dirs[array_to_str(x)]
            if theta < -180:
                theta += 360
            if theta > +180:
                theta -= 360
            r.append(('rotate', [theta]))
            p = x
            c = 1
    current_position = [current_position[0] + p[0] * c * _y, current_position[1] + p[1] * c]
    r.append(('go_to', [round(0.5 - current_position[0] / 10.0 - 0.05, 2), round(current_position[1] / 10.0 + 0.05, 2)]))

    #r.append(('go_to', [round(0.5 - p[0] / 10.0 - 0.05, 2), round(p[1] / 10.0 + 0.05, 2)]))
    # r.append(('go_to', [p[1]*c, p[0]*c]))
    return r

class GA_path_control(PathControl):
    def __init__(self, path, motion, command_dict):
        super(GA_path_control, self).__init__(motion, command_dict)
        a = load_array(path, delimiter=',')
        r = compress_array(a, 4, 0, 0, True)

        for cmd in r:
            super(GA_path_control, self).add(cmd)

