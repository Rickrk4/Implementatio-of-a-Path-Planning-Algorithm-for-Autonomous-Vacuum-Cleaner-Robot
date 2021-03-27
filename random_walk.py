#
# path_control
#
from random import randrange


class RandomWalk:
    TAG = 'random_walk'

    def __init__(self, r, command_dict, sensors, env):
        self.robot = r
        self.motion = r.motion
        self.command_dict = command_dict
        self.sensors = sensors
        self.state = 'go'
        self.theta = 120
        self.counter = 0
        self.backward_time = 0#100#400
        self.env = env

    def set_random_rotation(self):
        degree = randrange(self.theta / 2, self.theta, 45)
        if degree > 180:
            degree = degree - 360
        if self.env.l_bump:
            degree *= -1
        self.command_dict['rotate_relative'].set_target(degree)

    def change_state(self):
        # print('change state')
        if self.state == 'go':
            self.state = 'stop'
            return
        if self.state == 'go' or self.state == 'stop':
            self.state = 'rotate_relative'
            self.set_random_rotation()
        else:
            self.state = 'go'

        self.command_dict[self.state].reset()

    def evaluate(self, delta_t):

        if self.state == 'go':
            if self.sensors[0].evaluate():
                print('fermiamoci')
                self.command_dict['go'].reset()
                self.state = 'stop'

                if self.backward_time > 0:
                    self.counter = self.backward_time
                return

        #Si sposta leggermente indietro per non avere il bumper attivo.
        if self.state == 'back':
            if self.counter == 0:
                self.state = 'stop'
                self.motion.evaluate_vw(1,0, delta_t)
                return
            else:
                self.counter -= 1
                self.motion.evaluate_vw(-1, 0, delta_t)
                #print('processiamo back')
                return

        if self.state == 'stop':
            if not self.motion.is_stopped():
                self.motion.evaluate_vw(0.0, 0.0, delta_t)
                return
            else:
                if self.backward_time > 0 and self.counter > 0:
                    self.state = 'back'
                    #print('siamo in back')
                    self.motion.evaluate_vw(-1, 0, delta_t)
                else:
                    self.change_state()
            return

        if not self.command_dict[self.state].target_got():
            self.command_dict[self.state].evaluate(delta_t)
        else:
            print('change state')
            self.change_state()
