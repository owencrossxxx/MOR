# -*- coding: utf-8 -*-

#######################################################################
####################       IMPORT           ###########################
import os
import sys
path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(path+'/../python') # TO CHANGE


# MOR IMPORT
from mor.gui import utility
from mor.reduction import ReduceModel
from mor.reduction.container import ObjToAnimate

#######################################################################
####################       PARAMETERS       ###########################

# Select Output Dir and original scene name & path
from PyQt4 import QtCore, QtGui
app = QtGui.QApplication(sys.argv)

originalScene = utility.openFileName('Select the SOFA scene you want to reduce')
outputDir = utility.openDirName('Select the directory tha will contain all the results')


###My SOFT ROBOT
nodeToReduce = '/pneu'
cavity = ObjToAnimate("pneu/cavity", incr=0.005,incrPeriod=10,rangeOfAction=0.05)
listObjToAnimate = [cavity]
addRigidBodyModes = [0,0,0]

#nodeToReduce = '/modelNode'
#nord = ObjToAnimate("modelNode/nord", incr=5,incrPeriod=10,rangeOfAction=40)
#sud = ObjToAnimate("modelNode/sud", incr=5,incrPeriod=10,rangeOfAction=40)
#est = ObjToAnimate("modelNode/est", incr=5,incrPeriod=10,rangeOfAction=40)
#ouest = ObjToAnimate("modelNode/ouest", incr=5,incrPeriod=10,rangeOfAction=40)
#listObjToAnimate = [nord,ouest,sud,est]
#addRigidBodyModes = [0,0,0]


# Tolerance
tolModes = 0.001
tolGIE =  0.05

# Optional
verbose = True
nbrCPU = 4

packageName = 'test'
addToLib = False

#######################################################################
####################      INITIALIZATION     ##########################
reduceMyModel = ReduceModel(    originalScene,  
                                nodeToReduce,
                                listObjToAnimate,
                                tolModes,tolGIE,
                                outputDir,
                                packageName = packageName,
                                addToLib = addToLib,
                                verbose = verbose,
                                addRigidBodyModes = addRigidBodyModes)


#######################################################################
####################       EXECUTION        ###########################
### TO PERFORM THE REDUCTION ALL AT ONCE:
reduceMyModel.performReduction()

### TO PERFORM THE REDUCTION STEP BY STEP:
####################    SOFA LAUNCHER       ##########################
#                                                                    #
#            PHASE 1 : Snapshot Database Computation                 #
#                                                                    #
#      We modify the original scene to do the first step of MOR :    #
#   we add animation to each actuators we want for our model         #
#   add a writeState componant to save the shaking resulting states  #
#                                                                    #
######################################################################
#reduceMyModel.phase1()


####################    PYTHON SCRIPT       ##########################
#                                                                    #
#  PHASE 2 : Computation of the reduced basis with SVD decomposition #
#                                                                    #
#      With the previous result we combine all the generated         #
#       state files into one to be able to extract from it           #
#                       the different mode                           #
#                                                                    #
######################################################################
#reduceMyModel.phase2()


####################    SOFA LAUNCHER       ##########################
#                                                                    #
#            PHASE 3 : Reduced Snapshot Computation                  #
#     to store projected FEM internal forces  contributions          #
#                                                                    #
#      We launch again a set of sofa scene with the sofa launcher    #
#      with the same previous arguments but with a different scene   #
#      This scene take the previous one and add the model order      #
#      reduction component:                                          #
#            - HyperReducedFEMForceField                             #
#            - MappedMatrixForceFieldAndMass                         #
#            - ModelOrderReductionMapping                            #
#       and produce an Hyper Reduced description of the model        #
#                                                                    #
######################################################################
#reduceMyModel.phase3()


####################    PYTHON SCRIPT       ##########################
#                                                                    #
# PHASE 4 :  Computation of the reduced integration domain           #
#                in terms of elements and nodes                      #
#                                                                    #
#      Final step : we gather again all the results of the           #
#      previous scenes into one and then compute the RID and Weigts  #
#      with it. Additionnally we also compute the Active Nodes       #
#                                                                    #
######################################################################
#reduceMyModel.phase4()
