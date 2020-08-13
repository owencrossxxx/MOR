import Sofa

import os
path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'
#from stlib.physics.rigid import Floor

# Units: mm, kg, s.     Pressure in kPa = k (kg/(m.s^2)) = k (g/(mm.s^2) =  kg/(mm.s^2)

def createScene(rootNode):
    
                rootNode.createObject('RequiredPlugin',name='SoftRobots', pluginName='SoftRobots')
                rootNode.createObject('RequiredPlugin',name='SofaPython', pluginName='SofaPython')
                rootNode.createObject('RequiredPlugin',name='ModelOrderReduction', pluginName='ModelOrderReduction')
                rootNode.findData('gravity').value='9810 0 0'
                rootNode.createObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')

                rootNode.createObject('FreeMotionAnimationLoop')
                rootNode.createObject('GenericConstraintSolver', printLog='0', tolerance="1e-15", maxIterations="5000")
                rootNode.createObject('CollisionPipeline', verbose="0")
                rootNode.createObject('BruteForceDetection', name="N2")
                rootNode.createObject('RuleBasedContactManager', name="Response", response="FrictionContact", rules="0 * FrictionContact?mu=0.5" )
                rootNode.createObject('CollisionResponse', response="FrictionContact", responseParams="mu=0.7")
                rootNode.createObject('LocalMinDistance', name="Proximity", alarmDistance="2.5", contactDistance="0.5", angleCone="0.01")

                rootNode.createObject('BackgroundSetting', color='0 0.168627 0.211765')
                rootNode.createObject('OglSceneFrame', style="Arrows", alignment="TopRight")
                
                         
                ##########################################
                # FEM Model                              
                pneu = rootNode.createChild('pneu')
                pneu.createObject('EulerImplicit', name='odesolver',firstOrder="false", rayleighStiffness='0.1', rayleighMass='0.1')
                pneu.createObject('SparseLDLSolver', name='preconditioner', template="CompressedRowSparseMatrix3d")
                pneu.createObject('MeshVTKLoader', name='loader', filename='/home/owen/Softwares/plugins/ModelOrderReduction/tools/sofa_test_scene/mesh/T50.vtu',translation=[100,0,0], rotation = [0,-90,0])
                pneu.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
                pneu.createObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5', rx='0')
                pneu.createObject('UniformMass', totalMass='0.0378')
                pneu.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.45',  youngModulus='330.8', drawAsEdges="1")                    
                #pneu.createObject('BoxROI', name='boxROISubTopo', box='0 0 0 150 -100 1', drawBoxes='true') 
                #pneu.createObject('BoxROI', name='membraneROISubTopo', box='0 0 -0.1 150 -100 0.1',computeTetrahedra="false", drawBoxes='true') 
                pneu.createObject('GenericConstraintCorrection', solverName='preconditioner')



                ##########################################
                # Sub topology                           
                #pneu.createObject('BoxROI', name='boxROISubTopo', box='70 -5 -10 78 70 70', drawBoxes='0')
                pneu.createObject('BoxROI', name='membraneROISubTopo', box='70 -5 -10 78 70 70',computeTetrahedra="false", drawBoxes='1')
                modelSubTopo1 = pneu.createChild('modelSubTopo1')
                #modelSubTopo1.createObject('Mesh', position='@loader.position', tetrahedra="@boxROISubTopo1.tetrahedraInROI", name='container')
                #modelSubTopo1.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.49',  youngModulus='50000')
                modelSubTopo1.createObject('TriangleSetTopologyContainer', position='@../membraneROISubTopo.pointsInROI', triangles='@../membraneROISubTopo.trianglesInROI', name='container')
                modelSubTopo1.createObject('TriangleFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.49',  youngModulus='50000')

                #modelSubTopo = pneu.createChild('modelSubTopo')
                #modelSubTopo.createObject('Mesh', position='@loader.position', tetrahedra="@boxROISubTopo.tetrahedraInROI", name='container')
                #modelSubTopo.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',  youngModulus='1e12')
                
                pneu.createObject('BoxROI', name='boxROI', box='3 -5 -10 8 70 70', drawBoxes= '1')
                pneu.createObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12', angularStiffness='1e12')
                #pneu.createObject('LinearSolverConstraintCorrection', solverName='directSolver')

		##########################################
                # Constraint                             
                cavity = pneu.createChild('cavity')
                cavity.createObject('MeshSTLLoader', name='cavityLoader', filename='/home/owen/Softwares/plugins/ModelOrderReduction/tools/sofa_test_scene/mesh/T5Cav.STL', translation=[100,1,1], rotation = [0,-90,0])
                cavity.createObject('Mesh', src='@cavityLoader', name='topo')
                cavity.createObject('MechanicalObject', name='cavity')
                cavity.createObject('SurfacePressureConstraint', name="SurfacePressureConstraint", template='Vec3d', value="0", triangles='@topo.triangles', drawPressure='0', drawScale='0.0002', valueType="pressure")
                cavity.createObject('BarycentricMapping', name='mapping',  mapForces='false', mapMasses='false')
                                
                
                                
                modelCollis = pneu.createChild('modelCollis')
		modelCollis.createObject('MeshSTLLoader', name='loader', filename='/home/owen/Softwares/plugins/ModelOrderReduction/tools/sofa_test_scene/mesh/T5.STL', translation=[100,0,0], rotation = [0,-90,0])
		modelCollis.createObject('TriangleSetTopologyContainer', src='@loader', name='container')
		modelCollis.createObject('MechanicalObject', name='collisMO', template='Vec3d')
		modelCollis.createObject('Triangle',group="0")
		modelCollis.createObject('Line',group="0")
		modelCollis.createObject('Point',group="0")
		modelCollis.createObject('BarycentricMapping')
                
                
		##########################################
                # Visualization  
                modelVisu = pneu.createChild('visu')
                modelVisu.createObject(  'MeshSTLLoader', name= 'loader', filename='/home/owen/Softwares/plugins/ModelOrderReduction/tools/sofa_test_scene/mesh/T5.STL', translation=[100,0,0], rotation = [0,-90,0])

                modelVisu.createObject(  'OglModel',
                                    src='@loader',
                                    template='Vec3d',
                                    color='0.7 0.7 0.7 0.6')

                modelVisu.createObject('BarycentricMapping')

                #Floor(rootNode, translation=[-5,30,30], rotation = [0,0,90] , isAStaticObject=True)

                planeNode = rootNode.createChild('Plane')
		planeNode.createObject('MeshObjLoader', name='loader', filename="mesh/floorFlat.obj", triangulate="true")
		planeNode.createObject('Mesh', src="@loader")
		planeNode.createObject('MechanicalObject', src="@loader", rotation="0 0 90", translation="0 35 -1", scale="15")
		planeNode.createObject('Triangle',simulated="0", moving="0",group="1")
		planeNode.createObject('Line',simulated="0", moving="0",group="1")
		planeNode.createObject('Point',simulated="0", moving="0",group="1")
		planeNode.createObject('OglModel',name="Visual", fileMesh="mesh/floorFlat.obj", color="1 0 0 1",rotation="0 0 90", translation="0 35 -1", scale="15")
		planeNode.createObject('UncoupledConstraintCorrection')

                return rootNode
