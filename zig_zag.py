#
# path_control
#
from random import randrange
from environment import next_point

## TODO: se nel bel mezzo della rotazione incontriamo una collisione, dobbiamo invertire il senso delle rotazioni.

class ZigZag:
    TAG = 'zig_zag'

    def __init__(self, r, command_dict, sensors, env):
        self.robot = r
        self.motion = r.motion
        self.command_dict = command_dict
        self.sensors = sensors

        #self.state = 'go'
        self.state = "zero"
        self.internal_state = '1'
        self.counter = 100
        self.rotation_sense = 1

        self.invert_sense = False
        self.ignore_collision_timer = 0
        self.ignore_collision_time = 100 #0.1 sec



    def change_state(self, delta_t):

        if self.state == 'go':
            self.state = 'back'
            self.motion.evaluate_vw(-1, 0, delta_t)
            return
            self.state = 'rotate_relative'
            self.command_dict['rotate_relative'].set_target((+90 + self.getAlignError()) * self.rotation_sense)
            ## Pigro tentativo di cmabiare senso di marcia
            if self.internal_state == "rotation":
                self.command_dict['rotate_relative'].set_target(+180 * self.rotation_sense)
            return

        if self.state == 'rotate_relative':
            if self.internal_state == 'rotation':
                self.internal_state = 'go_until'
                self.rotation_sense *= -1
            else:
                self.internal_state = 'rotation'
                self.counter = 1000

            self.state = 'go'
            self.command_dict['go'].reset()
        #self.command_dict[self.state].reset()
        #print('current state:', self.state, self.internal_state)


    def getAlignError(self):
        self.sensors[3].evaluate(resolution=10000, absolute=False)
        min = 0
        min_value = self.sensors[3].memory[0]
        for i in range(0, 360):
            if self.sensors[3].memory[i] < min_value:
                min = i
                min_value = self.sensors[3].memory[i]

        if min > 180:
            min = min - 360
        return min
    def evaluate(self, delta_t):

        if self.state == "zero":
            if not self.command_dict['go'].target_got() and not self.sensors[0].evaluate():
                self.command_dict['go'].evaluate(delta_t)
                return
            else:
                theta = self.getAlignError()
                self.command_dict['rotate_relative'].set_target(theta)
                self.state = "align"

        if self.state == "align":
            #print('state = align')
            if self.command_dict["rotate_relative"].target_got():
                self.state = "go"
            else:
                self.command_dict["rotate_relative"].evaluate(delta_t)
            return


        if self.state == 'back':
            self.motion.evaluate_vw(1,0,delta_t)
            self.state = 'stop'

        if self.state == 'stop':
            self.motion.evaluate_vw(0,0,delta_t)
            if self.motion.is_stopped():
                self.state = 'rotate_relative'
                if self.internal_state != "rotation":
                    theta = self.getAlignError()
                else:
                    theta = 0
                print('correggiamo di', theta)

                if self.invert_sense:
                    self.invert_sense = False
                    self.command_dict['rotate_relative'].set_target((+90) * self.rotation_sense)
                else:
                    self.command_dict['rotate_relative'].set_target((+90) * self.rotation_sense + theta)

        if self.state == 'go':

            if self.internal_state == 'rotation':
                if self.counter > 0:
                    self.counter -= 1
                else:
                    self.command_dict['go'].reset()
                    self.state = 'stop'
                    return


            if not self.command_dict['go'].target_got() and not self.sensors[0].evaluate():
                self.command_dict['go'].evaluate(delta_t)
            else:
                # se siamo qui vuol dire che go ha incontrato un'ostacolo
                self.invert_sense = self.internal_state == "rotation"
                if self.invert_sense:
                    self.rotation_sense *= 1
                self.change_state(delta_t)

        if self.state == 'rotate_relative':
            if not self.command_dict['rotate_relative'].target_got():
                self.command_dict['rotate_relative'].evaluate(delta_t)
            else:
                self.change_state(delta_t)
