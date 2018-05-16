import umatrix
import urandom
from math import log, cos, sin, sqrt, pi


class ACapteur (object):
   def __init__(self, name_of_mes, moy, cho):
      self._name_of_mes = name_of_mes
      self._moy = moy.copy()
      self._cho = cho.copy()
   
   def get_num_mes(self):
      return len(self._moy)
      
   def comportement(self, x, u, t):
      u'''Loi de comportement des capteurs
      
      @param x \a array
         Vecteur d'etat du systeme
      
      @param u \a array
         Vecteur des commandes
      
      @param t (s) \a float
         Date courante
      
      @return mes \a array
         Vecteur de mesures bruitee suivant la loi gaussienne donnee a l'instanciation
         
      '''
      raise SystemError(u"[ERREUR]Methode comportement non implementee pour le capteur '%s'" % self._name)
      
   def mesure(self, x, u, t):
      z = self.comportement(x, u, t)
      
      n = len(self._name_of_mes)
      n2 = int((n+1)/2)*2
      # Vecteur de nombre aleatoires uniformes entre 0 et 1
      vu = [urandom.getrandbits(8*4)/float(2**32)]*n2
      for i in range(n2/2):
         vu[2*i+0],vu[2*i+1] = sqrt(-log(vu[2*i+0]))*cos(2*pi*vu[2*i+1]), sqrt(-log(vu[2*i+0]))*sin(2*pi*vu[2*i+1])
         
      bn = umatrix.matrix([vu[:n]]).T
      bg = self._cho*bn + self._moy
      
      self._mes = z + bg
   
   def get_mesurement(self):
      return self._mes
      
   def get_mes_generator(self):
      X = self.get_mesurement()
      for i in range(self.get_num_mes()):
         yield self._name_of_mes[i], X[i,0]
      
      
   
