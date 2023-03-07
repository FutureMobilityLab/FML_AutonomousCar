from getPath import getPath
from matplotlib import pyplot as plt
import numpy as np

def main ():
   [xref,yref,psiref] = getPath("waypoints.json")
   plt.plot(xref,yref)
   psiref_x = np.cos(psiref)
   psiref_y = np.sin(psiref)
   plt.quiver(xref,yref,psiref_x,psiref_y)
   plt.show()

if __name__ == "__main__":
   main()