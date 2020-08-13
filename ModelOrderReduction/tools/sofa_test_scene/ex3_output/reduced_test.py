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
                  nbrOfModes=3,
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

    pneu_MOR = modelRoot.createChild('pneu_MOR')
    pneu_MOR.createObject('EulerImplicit' , firstOrder = 'false', rayleighStiffness = '0.1', name = 'odesolver', rayleighMass = '0.1')
    pneu_MOR.createObject('SparseLDLSolver' , name = 'preconditioner', template = 'CompressedRowSparseMatrix3d')
    pneu_MOR.createObject('GenericConstraintCorrection' , solverName = 'preconditioner')
    pneu_MOR.createObject('MechanicalObject' , position = [0]*nbrOfModes, template = 'Vec1d')
    pneu_MOR.createObject('MechanicalMatrixMapperMOR' , object1 = '@./MechanicalObject', object2 = '@./MechanicalObject', listActiveNodesPath = path + r'/data/listActiveNodes.txt', template = 'Vec1d,Vec1d', usePrecomputedMass = True, timeInvariantMapping2 = True, performECSW = hyperReduction, timeInvariantMapping1 = True, precomputedMassPath = path + r'/data/UniformMass_reduced.txt', nodeToParse = '@./pneu')


    pneu = pneu_MOR.createChild('pneu')
    pneu.createObject('MeshVTKLoader' , scale3d = multiply(scale,[1.0, 1.0, 1.0]), translation = add(translation,[0, 0, 0]), rotation = add(rotation,[0, 0, 0]), name = 'loader', filename = path + r'/mesh/Ex30.vtu')
    pneu.createObject('TetrahedronSetTopologyContainer' , src = '@loader', name = 'container')
    pneu.createObject('MechanicalObject' , showIndices = 'false', rx = '0', showIndicesScale = '4e-5', name = 'tetras', template = 'Vec3d')
    pneu.createObject('UniformMass' , totalMass = '0.0378')
    pneu.createObject('HyperReducedTetrahedronFEMForceField' , RIDPath = path + r'/data/reducedFF_pneu_0_RID.txt', name = 'reducedFF_pneu_0', weightsPath = path + r'/data/reducedFF_pneu_0_weight.txt', youngModulus = '330.8', modesPath = path + r'/data/modes.txt', template = 'Vec3d', performECSW = hyperReduction, nbModes = nbrOfModes, method = 'large', poissonRatio = '0.45', drawAsEdges = '1')
    pneu.createObject('BoxROI' , name= 'membraneROISubTopo' , orientedBox= newBox([[81.0, 60.0, -10.0], [81.0, -5.0, -10.0], [85.0, -5.0, -10.0]] , [0, 0, 0],translation,rotation,[0, 0, 35.0],scale) + multiply(scale[2],[70.0]).tolist(),drawBoxes=True)
    pneu.createObject('BoxROI' , name= 'boxROI' , orientedBox= newBox([[-5.0, 60.0, -10.0], [-5.0, -5.0, -10.0], [3.0, -5.0, -10.0]] , [0, 0, 0],translation,rotation,[0, 0, 35.0],scale) + multiply(scale[2],[70.0]).tolist(),drawBoxes=True)
    pneu.createObject('HyperReducedRestShapeSpringsForceField' , RIDPath = path + r'/data/reducedFF_pneu_2_RID.txt', name = 'reducedFF_pneu_2', weightsPath = path + r'/data/reducedFF_pneu_2_weight.txt', points = '@boxROI.indices', modesPath = path + r'/data/modes.txt', stiffness = '1e8', performECSW = hyperReduction, nbModes = nbrOfModes)
    pneu.createObject('ModelOrderReductionMapping' , input = '@../MechanicalObject', modesPath = path + r'/data/modes.txt', output = '@./tetras')


    modelSubTopo1 = pneu.createChild('modelSubTopo1')
    modelSubTopo1.createObject('TriangleSetTopologyContainer' , position = '@../membraneROISubTopo.pointsInROI', name = 'container', triangles = '@../membraneROISubTopo.trianglesInROI')
    modelSubTopo1.createObject('HyperReducedTriangleFEMForceField' , RIDPath = path + r'/data/reducedFF_modelSubTopo1_1_RID.txt', name = 'reducedFF_modelSubTopo1_1', weightsPath = path + r'/data/reducedFF_modelSubTopo1_1_weight.txt', youngModulus = '50000', modesPath = path + r'/data/modes.txt', template = 'Vec3d', performECSW = hyperReduction, method = 'large', poissonRatio = '0.49', nbModes = nbrOfModes)


    cavity = pneu.createChild('cavity')
    cavity.createObject('MeshSTLLoader' , scale3d = multiply(scale,[1.0, 1.0, 1.0]), translation = add(translation,[0.5, 0.5, 2.5]), rotation = add(rotation,[0, 0, 0]), name = 'cavityLoader', filename = path + r'/mesh/Ex3Cav.STL')
    cavity.createObject('Mesh' , src = '@cavityLoader', name = 'topo')
    cavity.createObject('MechanicalObject' , name = 'cavity')
    cavity.createObject('SurfacePressureConstraint' , drawScale = '0.0002', name = 'SurfacePressureConstraint', valueType = 'pressure', value = '0.0', drawPressure = '0', template = 'Vec3d', triangles = '@topo.triangles')
    cavity.createObject('BarycentricMapping' , mapMasses = 'false', name = 'mapping', mapForces = 'false')


    modelCollis = pneu.createChild('modelCollis')
    modelCollis.createObject('MeshSTLLoader' , scale3d = multiply(scale,[1.0, 1.0, 1.0]), translation = add(translation,[0, 0, 0]), rotation = add(rotation,[0, 0, 0]), name = 'loader', filename = path + r'/mesh/Ex3.STL')
    modelCollis.createObject('TriangleSetTopologyContainer' , src = '@loader', name = 'container')
    modelCollis.createObject('MechanicalObject' , name = 'collisMO', template = 'Vec3d')
    modelCollis.createObject('Triangle' , group = '0')
    modelCollis.createObject('Line' , group = '0')
    modelCollis.createObject('Point' , group = '0')
    modelCollis.createObject('BarycentricMapping')


    visu = pneu.createChild('visu')
    visu.createObject('MeshSTLLoader' , scale3d = multiply(scale,[1.0, 1.0, 1.0]), translation = add(translation,[0.0, 0.0, 0.0]), rotation = add(rotation,[0.0, 0.0, 0.0]), name = 'loader', filename = path + r'/mesh/Ex3.STL')
    visu.createObject('OglModel' , color = '0.7 0.7 0.7 0.6', src = '@loader', template = 'Vec3d')
    visu.createObject('BarycentricMapping')

    return pneu


#   STLIB IMPORT
from stlib.scene import MainHeader
def createScene(rootNode):
    surfaceMeshFileName = False

    MainHeader(rootNode,plugins=["SofaPython","SoftRobots","ModelOrderReduction"],
                        dt=0.01,
                        gravity=[0.0, -9.81, 0.0])
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
