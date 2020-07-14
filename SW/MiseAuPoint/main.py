import numpy as np
import matplotlib.pyplot as plt

from SystemControl.Simulation import Simulation
from SystemControl.Controller import PIDController, LQRegulator
from SystemControl.SetPoint import Step
from SystemControl.Graphics import plotFromLogger

from FiltreKalmanGyro import FiltreKalmanGyro
from PenduleGyro import PenduleGyro
from Capteur import Capteur


def valide_modele_kal():
    t = 0.0

    # Pas de temps
    dt = 0.1

    # Angle initial
    th0 = 5 * np.pi / 180.0

    # Nombre de pas de temps simulés
    ns = 100
    # ns = 3

    sys = PenduleGyro()
    sys.set_state(np.matrix([th0, 0.0, np.pi / 2.0]).T, t)
    u = np.matrix([[-sys.A * th0 / sys.K]])

    kal = FiltreKalmanGyro()
    kal.set_state(
        np.matrix([th0, 0.0, 0.0]).T, np.matrix(np.diag((1.0, 1.0, 1.0))) / 10.0, t
    )
    kal.set_matrices(sys.A, sys.K, dt)

    log = Loggeur()

    for i in range(ns):
        xs, _ = sys.get_state()
        xk, _ = kal.get_estimation()

        log.log("t", t)
        log.log("xs", xs[0, 0] * 180.0 / np.pi)
        log.log("xk", xk[0, 0] * 180.0 / np.pi)

        t += dt

        kal.prediction(u)
        sys.integre_pas(u, dt)

    fig = plt.figure()

    axe = fig.add_subplot(111)
    axe.grid(True)
    log.plot("t", "xs", axe, label=u"integ")
    log.plot("t", "xk", axe, label=u"kal")
    # log.plot('t', 'xk-xs', axe, label=u'kal-integ')
    axe.legend(loc="best")

    plt.show()


def main():
    # Pas de temps
    dt = 0.1

    # Angle initial
    th0 = np.pi / 50

    sys = PenduleGyro()
    sys.setInitialState(np.array([th0, 0.0, np.pi / 2.0]))

    kal = FiltreKalmanGyro()
    kal.setInitialState(np.array([th0, 0.0, 0.0]))
    kal.setInitialStateCovariance(np.diag((1.0, 1.0, 1.0)) / 10.0)
    kal.KA = sys.A
    kal.KU = sys.K

    cpt = Capteur()

    a = 2.0
    P = (3 * a ** 2 + sys.A) / sys.K
    I = a ** 3 / sys.K
    D = 3 * a / sys.K
    ctrl = PIDController("ctrl", name_of_outputs=["cmd"])
    ctrl.P = P
    ctrl.I = I
    ctrl.D = D

    # ctrl = LQRegulator("ctrl", name_of_outputs=["cmd"])
    # ctrl.A = np.array([[0, 1], [sys.A, 0]])
    # ctrl.B = np.array([[0],[sys.K]])
    # ctrl.C = np.array([[1,0]])
    # ctrl.D = np.array([[0]])
    # ctrl.Q = np.eye(2)
    # ctrl.R = np.eye(1)
    # ctrl.computeGain()

    # Init controleur
    x_cons = 0
    user = Step("spt", cons=np.array([x_cons]), name_of_outputs=["step"])

    # tps = np.arange(0.0, 10., dt)
    tps = np.arange(0.0, 5 * dt, dt)
    sim = Simulation()

    sim.addElement(user)
    sim.addElement(ctrl)
    sim.addElement(sys)
    sim.addElement(cpt)
    sim.addElement(kal)

    sim.linkElements(user, ctrl, src_data_name="output", dst_input_name="setpoint")
    sim.linkElements(ctrl, sys, src_data_name="output", dst_input_name="command")
    sim.linkElements(sys, cpt, src_data_name="output", dst_input_name="state")
    sim.linkElements(cpt, kal, src_data_name="output", dst_input_name="measurement")
    sim.linkElements(ctrl, kal, src_data_name="output", dst_input_name="command")
    sim.linkElements(kal, ctrl, src_data_name="state[0:2]", dst_input_name="estimation")

    sim.simulate(tps)
    log = sim.getLogger()

    t = log.getValue("t")
    u = log.getValue("cmd")
    ps = log.getValue("ps_sim")
    dpsi = u / np.sin(ps)

    # =============================================
    # Tracés
    # =============================================
    fig = plt.figure()

    axe = fig.add_subplot(211)
    axe.grid(True)
    plotFromLogger(log, "t", "ps_sim", axe, label=u"psi")
    axe.legend(loc="best")

    axe = fig.add_subplot(212, sharex=axe)
    axe.grid(True)
    axe.plot(t, dpsi, label=u"dpsi")
    axe.legend(loc="best")
    axe.set_xlabel(u"Temps (s)")

    fig.tight_layout()

    fig = plt.figure()

    axe = fig.add_subplot(231)
    axe.grid(True)
    plotFromLogger(log, "t", "th_est", axe, label=u"estimé", marker="o", linestyle="")
    plotFromLogger(log, "t", "th_sim", axe, label=u"simu")
    plotFromLogger(log, "t", "th_mes", axe, label=u"mesuré", marker="+", linestyle="")
    axe.legend(loc="best")

    axe = fig.add_subplot(234, sharex=axe)
    axe.grid(True)
    # plotFromLogger(log, 't', 'sig_th', axe, label=u'sigma th_sim')
    plotFromLogger(log, "t", "th_est-th_sim", axe, label=u"delta th_sim")
    axe.legend(loc="best")
    axe.set_xlabel(u"Temps (s)")

    axe = fig.add_subplot(232)
    axe.grid(True)
    plotFromLogger(log, "t", "dth_est", axe, label=u"estimé", marker="o", linestyle="")
    plotFromLogger(log, "t", "dth_sim", axe, label=u"simu")
    plotFromLogger(log, "t", "dth_mes", axe, label=u"mesuré", marker="+", linestyle="")
    axe.legend(loc="best")

    axe = fig.add_subplot(235, sharex=axe)
    axe.grid(True)
    # plotFromLogger(log, 't', 'sig_dth', axe, label=u'sigma dth_sim')
    plotFromLogger(log, "t", "dth_est-dth_sim", axe, label=u"delta dth_sim")
    axe.legend(loc="best")
    axe.set_xlabel(u"Temps (s)")

    axe = fig.add_subplot(233)
    axe.grid(True)
    plotFromLogger(
        log, "t", "biais_est", axe, label=u"estimé", marker="o", linestyle=""
    )
    # plotFromLogger(log, 't', 'biais', axe, label=u'simu')
    axe.legend(loc="best")

    axe = fig.add_subplot(236, sharex=axe)
    axe.grid(True)
    # plotFromLogger(log, 't', 'sig_biais', axe, label=u'sigma biais')
    # plotFromLogger(log, 't', 'biais_est-biais', axe, label=u'delta biais')
    axe.legend(loc="best")
    axe.set_xlabel(u"Temps (s)")

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
    # valide_modele_kal()
