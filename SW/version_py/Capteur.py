import umatrix
import ulinalg
from math import cos, sin, sqrt, pi

from libSystemControl.Capteur import ACapteur


# =============================================
# Définition des capteurs
# =============================================
class Capteur(ACapteur):
    def __init__(self, lbda):
        self.std_pos = 0.00448355705097 * 10
        self.std_vel = 0.0014239412603
        self.lbda = lbda

        moy = pi / 180.0 * umatrix.matrix([[-5.0, 4.0]]).T
        cho = umatrix.matrix([[self.std_pos, 0.0], [0.0, self.std_vel]])
        ACapteur.__init__(self, [u"pos_mes", "vel_mes"], moy, cho)

    def comportement(self, x, u, t):
        th = x[0, 0]
        dth = -self.lbda * (th - u[0, 0])
        return umatrix.matrix([[th, dth]]).T
