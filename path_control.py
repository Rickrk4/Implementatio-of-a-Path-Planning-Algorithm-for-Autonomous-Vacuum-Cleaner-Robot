#
# path_control
#

class PathControl:
    TAG = 'PathControl'

    def __init__(self, motion, command_dict):
        self.motion = motion
        self.command_dict = command_dict
        self.command_list = []
        self.current_command = 0
        self.running = False

    def add(self, command):
        self.command_list.append(command)

    def evaluate(self, delta_t):


        if self.current_command == len(self.command_list):
            self.motion.evaluate(0, 0, delta_t)
            self.running = False
            return
        if not (self.running):
            (cmd, args) = self.command_list[self.current_command]
            obj = self.command_dict[cmd]
            obj.set_target(*args)
            self.running = True
        else:
            (cmd, _) = self.command_list[self.current_command]
            obj = self.command_dict[cmd]
            if obj.target_got():
                self.current_command = self.current_command + 1
                print('nuovo comando', self.command_list[self.current_command])
                self.running = False
            else:
                obj.evaluate(delta_t)
