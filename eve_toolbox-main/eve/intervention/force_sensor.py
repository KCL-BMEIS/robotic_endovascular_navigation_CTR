import Sofa
import Sofa.Gui
import Sofa.Components
from Sofa.constants import *
import numpy as np

from OpenGL.GL import *
from OpenGL.GLU import *


class CollisionMonitor(Sofa.Core.Controller):
    def __init__(self, node, name, MO, numberNodes, numberDOFs, constraintSolver, verbose=False, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.name = name
        self.MO = MO
        self.numberNodes = numberNodes
        self.numberDOFs = numberDOFs
        self.constraintSolver = constraintSolver
        self.verbose = verbose
        print('init collision monitor')

    def init(self):
        pass

    def bwdInit(self):
        pass

    def reinit(self):
        pass

    def onAnimateBeginEvent(self, dt):
        
        self.sortedCollisionMatrix = [[0, 0, 0] for i in range(self.numberNodes)]

        self.collisionMatrix = self.MO.constraint.value.splitlines()
        self.collisionMatrix = [line.split() for line in self.collisionMatrix]
        self.collisionMatrix = [[float(Value) for Value in self.collisionMatrix[i]] for i in
                                range(len(self.collisionMatrix))]
        # print(self.collisionMatrix) # test

        for i in range(len(self.collisionMatrix)):
            numberElements = self.collisionMatrix[i][1]

            if numberElements == 1:
                element1 = int(self.collisionMatrix[i][2])
                self.sortedCollisionMatrix[element1] = [self.sortedCollisionMatrix[element1][j] + self.collisionMatrix[i][3 + j] for j in range(3)]

            if numberElements == 2:
                element1 = int(self.collisionMatrix[i][2])
                element2 = int(self.collisionMatrix[i][3 + self.numberDOFs])
                self.sortedCollisionMatrix[element1] = [self.sortedCollisionMatrix[element1][j] + self.collisionMatrix[i][3 + j] for j in range(3)]
                self.sortedCollisionMatrix[element2] = [self.sortedCollisionMatrix[element2][j] + self.collisionMatrix[i][4 + self.numberDOFs + j] for j in range(3)]

            if numberElements == 3:
                element1 = int(self.collisionMatrix[i][2])
                element2 = int(self.collisionMatrix[i][3 + self.numberDOFs])
                element3 = int(self.collisionMatrix[i][4 + 2 * self.numberDOFs])
                self.sortedCollisionMatrix[element1] = [self.sortedCollisionMatrix[element1][j] + self.collisionMatrix[i][3 + j] for j in range(3)]
                self.sortedCollisionMatrix[element2] = [self.sortedCollisionMatrix[element2][j] + self.collisionMatrix[i][4 + self.numberDOFs + j] for j in range(3)]
                self.sortedCollisionMatrix[element3] = [self.sortedCollisionMatrix[element3][j] + self.collisionMatrix[i][5 + 2 * self.numberDOFs + j] for j in range(3)]

        # for i in range(len(self.sortedCollisionMatrix)):
        #    self.sortedCollisionMatrix[i] = np.linalg.norm(np.array(self.sortedCollisionMatrix[i]))

        if (self.verbose):
            print(self.sortedCollisionMatrix)
