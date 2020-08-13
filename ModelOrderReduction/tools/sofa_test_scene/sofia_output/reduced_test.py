# -*- coding: utf-8 -*-
import os
import Sofa
from numpy import add,subtract,multiply
try:
    from splib.numerics import *
except:
    raise ImportError("ModelOrderReduction plugin depend on SPLIB"\
                     +"Please install it : https://github.com/SofaDefrost/STLIB")

path = os.path.dirname(os.path.abspath(__file__))

def TRSinOrigin(positions,modelPosition,translation,rotation,scale=[1.0,1.0,1.0]):
    posOrigin = subtract(positions , modelPosition)
    if any(isinstance(el, list) for el in positions):
        posOriginTRS = transformPositions(posOrigin,translation,eulerRotation=rotation,scale=scale)
    else:
        posOriginTRS = transformPosition(posOrigin,TRS_to_matrix(translation,eulerRotation=rotation,scale=scale))
    return add(posOriginTRS,modelPosition).tolist()
    
def newBox(positions,modelPosition,translation,rotation,offset,scale=[1.0,1.0,1.0]):
    pos = TRSinOrigin(positions,modelPosition,translation,rotation,scale)
    offset =transformPositions([offset],eulerRotation=rotation,scale=scale)[0]
    return add(pos,offset).tolist()

def Reduced_test(
                  attachedTo=None,
                  name="Reduced_test",
                  rotation=[0.0, 0.0, 0.0],
                  translation=[0.0, 0.0, 0.0],
                  scale=[1.0, 1.0, 1.0],
                  surfaceMeshFileName=False,
                  surfaceColor=[1.0, 1.0, 1.0],
                  nbrOfModes=7,
                  hyperReduction=True):
    """
    Object with an elastic deformation law.

        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | argument            | type      | definition                                                                                      |
        +=====================+===========+=================================================================================================+
        | attachedTo          | Sofa.Node | Where the node is created;                                                                      |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | name                | str       | name of the Sofa.Node it will                                                                   |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | rotation            | vec3f     | Apply a 3D rotation to the object in Euler angles.                                              |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | translation         | vec3f     | Apply a 3D translation to the object.                                                           |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | scale               | vec3f     | Apply a 3D scale to the object.                                                                 |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | surfaceMeshFileName | str       | Filepath to a surface mesh (STL, OBJ). If missing there is no visual properties to this object. |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | surfaceColor        | vec3f     | The default color used for the rendering of the object.                                         |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | nbrOfModes          | int       | Number of modes we want our reduced model to work with                                          |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+
        | hyperReduction      | Bool      | Controlled if we have the simple reduction or the hyper-reduction                               |
        +---------------------+-----------+-------------------------------------------------------------------------------------------------+

    """

    modelRoot = attachedTo.createChild(name)

    SofiaLeg_MOR = modelRoot.createChild('SofiaLeg_MOR')
    SofiaLeg_MOR.createObject('EulerImplicit' , firstOrder = '0', name = 'odesolver')
    SofiaLeg_MOR.createObject('SparseLDLSolver' , name = 'preconditioner', template = 'CompressedRowSparseMatrixd')
    SofiaLeg_MOR.createObject('MechanicalObject' , position = [0]*nbrOfModes, template = 'Vec1d')
    SofiaLeg_MOR.createObject('MechanicalMatrixMapperMOR' , object1 = '@./MechanicalObject', object2 = '@./MechanicalObject', listActiveNodesPath = path + r'/data/listActiveNodes.txt', template = 'Vec1d,Vec1d', usePrecomputedMass = True, timeInvariantMapping2 = True, performECSW = hyperReduction, timeInvariantMapping1 = True, precomputedMassPath = path + r'/data/UniformMass_reduced.txt', nodeToParse = '@./SofiaLeg')


    SofiaLeg = SofiaLeg_MOR.createChild('SofiaLeg')
    SofiaLeg.createObject('MeshVTKLoader' , scale3d = multiply(scale,[1.0, 1.0, 1.0]), translation = add(translation,[0.0, 0.0, 0.0]), rotation = add(rotation,[0.0, 0.0, 0.0]), name = 'loader', filename = path + r'/mesh/sofia_leg.vtu')
    SofiaLeg.createObject('TetrahedronSetTopologyContainer' , position = '@loader.position', createTriangleArray = '1', name = 'container', tetrahedra = '@loader.tetrahedra', checkConnexity = '1')
    SofiaLeg.createObject('MechanicalObject' , showIndices = 'false', showIndicesScale = '4e-5', name = 'tetras', template = 'Vec3d', position = '@loader.position')
    SofiaLeg.createObject('UniformMass' , totalMass = 0.01)
    SofiaLeg.createObject('HyperReducedTetrahedronFEMForceField' , RIDPath = path + r'/data/reducedFF_SofiaLeg_0_RID.txt', name = 'reducedFF_SofiaLeg_0', weightsPath = path + r'/data/reducedFF_SofiaLeg_0_weight.txt', youngModulus = 300, modesPath = path + r'/data/modes.txt', performECSW = hyperReduction, poissonRatio = 0.45, nbModes = nbrOfModes)
    SofiaLeg.createObject('BoxROI' , orientedBox = [[-12.0, 53.0, 0.0], [12.0, 53.0, 0.0], [12.0, 64.0, 0.0], 16.0], drawBoxes = True, name = 'boxROITop')
    SofiaLeg.createObject('HyperReducedRestShapeSpringsForceField' , RIDPath = path + r'/data/reducedFF_SofiaLeg_1_RID.txt', name = 'reducedFF_SofiaLeg_1', weightsPath = path + r'/data/reducedFF_SofiaLeg_1_weight.txt', points = '@boxROITop.indices', modesPath = path + r'/data/modes.txt', printLog = True, stiffness = '1e8', performECSW = hyperReduction, nbModes = nbrOfModes)
    SofiaLeg.createObject('BoxROI' , orientedBox = [[-25.0, -41.0, -7.0], [25.0, -42.0, -7.0], [25.0, -39.0, -7.0], 2.0, [-25.0, -42.0, 7.0], [25.0, -42.0, 7.0], [25.0, -39.0, 7.0], 2.0], drawSize = 5, drawPoints = '0', drawBoxes = True, name = 'boxROICollision', computeTriangles = '0', computeQuad = '0', computeEdges = '0', computeTetrahedra = '0', computeHexahedra = '0')
    SofiaLeg.createObject('BoxROI' , orientedBox = [[-2.5, -8.5, 0.0], [2.5, -8.5, 0.0], [2.5, -3.5, 0.0], 18.0], drawBoxes = True, name = 'boxROIMiddle')
    SofiaLeg.createObject('HyperReducedRestShapeSpringsForceField' , external_points = [0, 1, 2], RIDPath = path + r'/data/reducedFF_SofiaLeg_2_RID.txt', name = 'reducedFF_SofiaLeg_2', weightsPath = path + r'/data/reducedFF_SofiaLeg_2_weight.txt', points = '@boxROIMiddle.indices', modesPath = path + r'/data/modes.txt', printLog = True, stiffness = '1e8', performECSW = hyperReduction, external_rest_shape = '@../../SofiaLeg_actuator/actuatorState', nbModes = nbrOfModes)
    SofiaLeg.createObject('ModelOrderReductionMapping' , input = '@../MechanicalObject', modesPath = path + r'/data/modes.txt', output = '@./tetras')


    SofiaLeg_actuator = modelRoot.createChild('SofiaLeg_actuator')
    SofiaLeg_actuator.createObject('MechanicalObject' , showObject = False, position = '@../SofiaLeg_MOR/SofiaLeg/boxROIMiddle.pointsInROI', name = 'actuatorState', template = 'Vec3d')


    Visual = SofiaLeg.createChild('Visual')
    Visual.createObject('MeshSTLLoader' , scale3d = multiply(scale,[1.0, 1.0, 1.0]), rotation = add(rotation,[0.0, 0.0, 0.0]), translation = add(translation,[0.0, 0.0, 0.0]), name = 'loader', filename = path + r'/mesh/sofia_leg.stl')
    Visual.createObject('OglModel' , color = [1.0, 1.0, 1.0], src = '@loader', template = 'ExtVec3f')
    Visual.createObject('BarycentricMapping')

    return SofiaLeg


#   STLIB IMPORT
from stlib.scene import MainHeader
def createScene(rootNode):
    surfaceMeshFileName = False

    MainHeader(rootNode,plugins=["SofaPython","SoftRobots","ModelOrderReduction"],
                        dt=0.01,
                        gravity=[0.0, -9810.0, 0.0])
    rootNode.VisualStyle.displayFlags="showForceFields"
    
    Reduced_test(rootNode,
                        name="Reduced_test",
                        surfaceMeshFileName=surfaceMeshFileName)

    # translate = 300
    # rotationBlue = 60.0
    # rotationWhite = 80
    # rotationRed = 70

    # for i in range(3):

    #     Reduced_test(rootNode,
    #                    name="Reduced_test_blue_"+str(i),
    #                    rotation=[rotationBlue*i, 0.0, 0.0],
    #                    translation=[i*translate, 0.0, 0.0],
    #                    surfaceColor=[0.0, 0.0, 1, 0.5],
    #                    surfaceMeshFileName=surfaceMeshFileName)
    # for i in range(3):

    #     Reduced_test(rootNode,
    #                    name="Reduced_test_white_"+str(i),
    #                    rotation=[0.0, rotationWhite*i, 0.0],
    #                    translation=[i*translate, translate, -translate],
    #                    surfaceColor=[0.5, 0.5, 0.5, 0.5],
    #                    surfaceMeshFileName=surfaceMeshFileName)

    # for i in range(3):

    #     Reduced_test(rootNode,
    #                    name="Reduced_test_red_"+str(i),
    #                    rotation=[0.0, 0.0, i*rotationRed],
    #                    translation=[i*translate, 2*translate, -2*translate],
    #                    surfaceColor=[1, 0.0, 0.0, 0.5],
    #                    surfaceMeshFileName=surfaceMeshFileName)
