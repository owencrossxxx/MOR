#!/usr/bin/env python
# -*- coding: utf-8 -*-
import Sofa
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

x = []
y = []
p = []
tmax = 500000

class controller(Sofa.PythonScriptController):

    def initGraph(self, node):

            self.node = node
            self.ReduceNode=self.node.getChild('Reduced_test')
            self.ModelNode=self.ReduceNode.getChild('pneu_MOR')
            self.pneu1Node=self.ModelNode.getChild('pneu')
            self.pressureConstraint1Node = self.pneu1Node.getChild('cavity')

            # create pointer towards the MechanicalObject
            self.myMechanicalObjectPointer = self.pneu1Node.getObject('tetras')
            #self.pneu1Node.getObject('FEM').findData('youngModulus').value = 100000000000

    def onBeginAnimationStep(self, dt):
        #do whatever you want at the beginning of the step
        global t
        global pressureValue
        global myMOpositions

        t = self.pneu1Node.findData('time').value
        incr = t*1000.0

        self.MecaObject1=self.pneu1Node.getObject('tetras')
        self.pressureConstraint1 = self.pressureConstraint1Node.getObject('SurfacePressureConstraint')

        myMOpositions = self.myMechanicalObjectPointer.findData('position').value
        pressureValue = self.pressureConstraint1.findData('value').value[0][0]
        
        #if t>0.2:
            #self.node.getRootContext().animate = False
            #self.pneu1Node.getObject('FEM').findData('youngModulus').value = 330
            #self.node.getRootContext().animate = True


        #DRL Patch communication
        



    def onKeyPressed(self,c):
            self.dt = self.node.findData('dt').value
            incr = self.dt*1000.0;

            self.MecaObject1=self.pneu1Node.getObject('tetras');

            self.pressureConstraint1 = self.pressureConstraint1Node.getObject('SurfacePressureConstraint')

            if (c == "="):
                pressureValue = self.pressureConstraint1.findData('value').value[0][0] + 0.005
                if pressureValue > 0.05:
                    pressureValue = 0.05
                self.pressureConstraint1.findData('value').value = str(pressureValue)

            if (c == "-"):
                pressureValue = self.pressureConstraint1.findData('value').value[0][0] - 0.005
                if pressureValue < -0.01:
                    pressureValue = -0.01
                self.pressureConstraint1.findData('value').value = str(pressureValue)

            #p.append(pressureValue)

    #called on each animation step
    #def onBeginAnimationStep(self, dt):
        #do whatever you want at the beginning of the step
        #t = self.rootNode.findData('time').value
        #return 0

    #called on each animation step
    def onEndAnimationStep(self, dt):

        # print the first value of the DOF 0 (Vec3 : x,y,z) x[0] y[0] z[0]
        #print str(t)
        #print str(myMOpositions[5783][0])+' '+str(myMOpositions[5783][1])+' '+str(myMOpositions[5783][2])

        p.append(pressureValue)
        x.append(t)
        y.append(myMOpositions[23][0])

        #print 'current state :', myMOpositions[23][0]

        if t>= tmax:
            #self.pneu1Node.getRootContext().animate = False

            plt.figure()

            plt.subplot(211)
            plt.plot(x, y)
            plt.yscale('linear')
            plt.ylabel('Displacement/mm')
            plt.grid(True)

            #l = [2*x for x in p]
            plt.subplot(212)
            plt.plot(x, p)
            plt.yscale('linear')
            plt.ylabel('Pressure/MPa')
            plt.xlabel('time')
            plt.grid(True)


            plt.show()

        return 0
