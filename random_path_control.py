import numpy as np

class RandomPathControl:
    def __init__(self, dictionary):
        self.dictionary = dictionary
        self.current = 0
        self.c = 0
        self.end = False
    def evaluate(self, delta_t):

        if self.c == self.dictionary[self.current][1] * np.reciprocal(delta_t):
            self.current = self.current +1
            self.c = 0
            print('cambio, adessio siamo a', self.current)
        else:
            self.c = (self.c + 1) % len(self.dictionary)
            if self.c == 0:
                self.end = True

        self.dictionary[self.current][0].evaluate(delta_t)