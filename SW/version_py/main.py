import umatrix
import ulinalg
from math import exp, pi

from libSystemControl.Estimateur import FiltreKalman
from libSystemControl.Simulation import ASimulation

from SystemeTest import SystemeTest
from Capteur import Capteur
from Controller import Controller


class Simulation(ASimulation):
    def comportement(self, x, t):
        return umatrix.matrix([[pi / 180.0 * 20]])


def main():
    # Cadence de calcul
    fs = 100.0

    # Init système
    lbda = 5.0
    sys = SystemeTest(lbda)
    sys.set_state(umatrix.matrix([[-pi / 4]]).T, 0.0)

    # Init Kalman
    A = umatrix.matrix([[exp(-lbda / fs), 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    B = umatrix.matrix([[1.0 - exp(-lbda / fs), 0.0, 0.0]]).T
    C = umatrix.matrix([[1.0, 1.0, 0.0], [-lbda, 0.0, 1.0]])
    D = umatrix.matrix([[0.0, lbda]]).T
    Q = 0 * ulinalg.eye(3)
    std_pos = 0.00448355705097
    std_vel = 0.0014239412603
    R = umatrix.matrix([[std_pos ** 2, 0.0], [0.0, std_vel ** 2]])

    kal = FiltreKalman(["pos_est", "biais_pos_est", "biais_vel_est"])
    kal.set_matrices(A, B, C, D, Q, R, 1.0 / fs)
    kal.set_state(umatrix.matrix([[0.0, 0.0, 0.0]]).T, ulinalg.eye(3) / 10.0, 0.0)

    c = Capteur(lbda)

    ctrl = Controller(1.0 / fs, sys)

    sim = Simulation(1.0 / fs, ctrl, sys, c, kal)

    sim.simule(20.0)
    log = sim.get_loggeur()


if __name__ == "__main__":
    main()
