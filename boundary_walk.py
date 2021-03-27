#
# path_control
#
from random import randrange
from environment import next_point


class BoundaryWalk:
    TAG = 'boundary_walk'

    def __init__(self, r, command_dict, sensors, env):
        self.robot = r
        self.motion = r.motion
        self.command_dict = command_dict
        self.sensors = sensors
        self.state = 'zero'#'go'
        self.internal_state = '0'
        self.backward_timer = 1
        self.forward_timer = 3000  # 3 sec

        self.counter = -1
        self.right_turn_counter = 0
        self.rotation_sense = 1

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

    def change_state(self):
        if self.state == 'go':
            if self.counter == 0:  # and self.internal_state != '0' :
                self.state = 'stop'
                self.command_dict['rotate_relative'].set_target(-90)
                self.right_turn_counter += 1
            else:
                alfa = self.sensors[3].evaluate()[1]
                if alfa > 180:
                    alfa -= 360

                #print('abbiamo sbattuto in direzione', alfa)
                #alfa è l'angolo di differenza per allinearsi all'ostacolo
                alfa = self.getAlignError()
                print('stiamo applicando la correzine di', alfa)

                # Applichiamo solo piccole correzioni
                if abs(alfa) > 30:
                    alfa = 0
                #else:
                #    alfa += 2



                self.command_dict['rotate_relative'].set_target(+90 + alfa)
                self.state = 'back'
                self.counter = self.backward_timer
                self.right_turn_counter = 0
            return
        if self.state == 'back':
            self.state = 'stop'
            return
        if self.state == 'stop':
            self.state = 'rotate_relative'
            return
        if self.state == 'rotate_relative':
            self.state = 'go'
            self.command_dict['go'].reset()
            self.counter = self.forward_timer
            return

    def evaluate(self, delta_t):

        if self.state == "zero":
            if not self.command_dict['go'].target_got():
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


        #print(self.right_turn_counter)
        if self.state == 'go':
            if self.counter != 0 or self.right_turn_counter >= 2:# or self.sensors[1].evaluate() < self.sensors[1].distance:  # or self.internal_state != '0':
                self.counter -= 1
            else:
                self.change_state()
                return

            if not self.command_dict['go'].target_got() and not self.sensors[0].evaluate():
                self.command_dict['go'].evaluate(delta_t)
            else:
                print('abbiamo colliso')
                self.change_state()
            return

        if self.state == 'back':
            if self.counter > 0:
                self.counter -= 1
                self.motion.evaluate_vw(-1, 0, delta_t)
            else:
                self.motion.evaluate_vw(1, 0, delta_t)
                self.change_state()
            return

        if self.state == 'stop':
            self.motion.evaluate_vw(0, 0, delta_t)
            if self.motion.is_stopped():
                self.change_state()
            return

        if self.state == 'rotate_relative':
            if not self.command_dict['rotate_relative'].target_got():
                self.command_dict['rotate_relative'].evaluate(delta_t)
            else:
                self.change_state()
            return
