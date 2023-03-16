import numpy as np
import os
import json
from pathlib import Path

def getPath(filename):
    track_file = os.path.join(str(Path(__file__).parent.parent.parent),'config', filename)
    f = open(track_file)
    waypoints = json.load(f)
    f.close()
    xref = np.array([x[0] for x in waypoints["smoothed_wpts"]])
    yref = np.array([x[1] for x in waypoints["smoothed_wpts"]])
    x_diffs = np.diff(xref)
    y_diffs = np.diff(yref)
    psiref = np.arctan2(y_diffs,x_diffs)
    psiref = np.append(psiref, psiref[-1])
    return xref,yref,psiref


if __name__ == "__main__":
    getPath()