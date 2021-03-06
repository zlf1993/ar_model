import numpy as np


class OBJ(object):
    def __init__(self, filename, swapyz=False):
        """Loads a Wavefront OBJ file. """
        # List of geometric vertices, with (x,y,z[,w]) coordinates, w is optional and defaults 1.0. v
        self.vertices = []
        # List of vertex normals in (x,y,z) form; normals might not be unit vectors. vn
        self.normals = []
        # List of texture coordinates, in (u, v [,w]) coordinates, these will vary between 0 and 1. vt
        self.texcoords = []
        # Polygonal face element (see below). f
        self.faces = []
        material = None
        for line in open(filename, "r"):
            if line.startswith('#'):
                continue
            values = line.split()
            if not values:
                continue
            if values[0] == 'v':
                v = np.array(values[1:4], dtype=np.float16)
                if swapyz:
                    v = v[0], v[2], v[1]
                self.vertices.append(v)
            elif values[0] == 'vn':
                v = values[1:4]
                if swapyz:
                    v = v[0], v[2], v[1]
                self.normals.append(v)
            elif values[0] == 'vt':
                self.texcoords.append(map(float, values[1:3]))
            elif values[0] == 'f':
                face = []
                texcoords = []
                norms = []
                for v in values[1:]:
                    w = v.split('/')
                    face.append(int(w[0]))
                    if len(w) >= 2 and len(w[1]) > 0:
                        texcoords.append(int(w[1]))
                    else:
                        texcoords.append(0)
                    if len(w) >= 3 and len(w[2]) > 0:
                        norms.append(int(w[2]))
                    else:
                        norms.append(0)
                self.faces.append((face, norms, texcoords))
