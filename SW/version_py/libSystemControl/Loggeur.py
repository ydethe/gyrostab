class Loggeur(object):
    def __init__(self):
        self.reset()

    def reset(self):
        self._data = {}
        self._fic = open("log.txt", "w")

    def log(self, nom, val):
        self._fic.write("%s, %f\n" % (nom, val))
