from sympy import *
init_printing(use_unicode=False)


def rotation(axe, angle):
   u'''Matrice de rotation
   
   '''
   u,v,w = axe
   return cos(angle)*eye(3) + (1-cos(angle))*axe*axe.T + sin(angle)*Matrix([[0, -w,  v],
                                                                            [w,  0, -u],
                                                                            [-v, u,  0]])

# =================================
# Définition des paramètres
# =================================
# Temps courant
t = Symbol('t')

# Dimensions du mandrin
r1,r2,h0 = symbols('r1 r2 h0')

# Moment d'inertie du pendule
J = Symbol('J')

# Masse du pendule
M = Symbol('M')

# Masse du mandrin
m = Symbol('m')

# Distance du point M courant à l'axe de rotation propre
r = Symbol('r')

# Angle du point M courant à l'axe de rotation propre
phi = Symbol('phi')

# Position du point M courant le long de l'axe de rotation propre
h = Symbol('h')

# Vitesse de rotation propre
w = Symbol('w')

# Angle de précession
psi = Function('psi')

# Masse volumique du mandrin
rho = Symbol('rho')

# Angle du système par rapport à la verticale.
# Un angle nul signifie que le pendule est sur son équilibre instable
theta = Function('theta')

# Gravité
g = Symbol('g')

# Distance pivol principal / centre de gravite
OG = Symbol('OG')

# Distance pivol principal / axe de rotation propre
OB = Symbol('OB')

dth2,dth1,dpsi2,th,dpsi1,ps = symbols('dth2 dth1 dpsi2 th dpsi1 ps')

# # Position du point courant dans le repère lié au gimbal interne
# # Le centre de ce repère est l'intersection axe de précession/axe de rotation principal
# # L'axe de précession est porté par X
# # L'axe du mandrin est porté par Z
# # Y complète le trièdre direct
# OM0 = Matrix([r*cos(phi+w*t)+OB, r*sin(phi+w*t), h])

# # Matrice de changement de repère gimbal interne --> gimbal externe
# # Le centre du repère lié au gimbal externe est l'intersection axe de précession/axe de rotation principal
# # L'axe de précession est porté par X
# # L'axe de rotation "principal" du gimbal externe par rapport au labo est porté par Z
# # Y complète le trièdre direct
# P1 = rotation(Matrix([1,0,0]), psi(t))

# # Matrice de changement de repère gimbal externe --> masse
# # Le centre du repère lié au gimbal externe est l'intersection axe de précession/axe de rotation principal
# # L'axe de précession est porté par X
# # L'axe de rotation "principal" du gimbal externe par rapport au labo est porté par Z
# # Y complète le trièdre direct
# P2 = rotation(Matrix([0,0,1]), theta(t))

# OM = P2*P1*OM0

# # Intégration du moment cinétique sur le mandrin
# mz = simplify((OM.cross(diff(OM,t,2)))[2])
# mom = simplify(integrate(integrate(integrate(mz*rho*r,(phi,0,2*pi)),(r,r1,r2)),(h,-h0/2,h0/2)))

# eq = mom+OG*g*M*sin(theta(t))-J*diff(theta(t),t,2)
# eq = eq.subs(diff(theta(t),t,2),dth2).subs(diff(theta(t),t),dth1).subs(diff(psi(t),t,2),dpsi2).subs(diff(psi(t),t),dpsi1).subs(theta(t),th).subs(psi(t),ps)
# dth2 = solve(eq,dth2)[0]
# dth2 = simplify(dth2.subs(rho,m/(h0*pi*(r2**2-r1**2))))
# print(dth2)

# Hypothèse : h0**2 = 3*r1**2 + 3*r2**2
dth2 = (6*M*OG*g*sin(th) - dpsi1*m*h0**2*w*sin(ps))/(6*J - 6*OB**2*m - m*h0**2)
print(simplify(dth2))


