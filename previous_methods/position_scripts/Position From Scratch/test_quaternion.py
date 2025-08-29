import imufusion as fus

import numpy as np
q = fus.Quaternion(np.array([1, 0, 0, 0]))

rotated = q.rotate([0, 0, 1])    # Should return [0, 0, 1]
print("Rotated vector:", rotated)
