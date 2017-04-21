# from pandac.PandaModules import loadPrcFileData
# loadPrcFileData('', 'load-display tinydisplay')

import sys
# import direct.directbase.DirectStart

from direct.showbase.DirectObject import DirectObject
from direct.directbase.DirectStart import base
from direct.showbase.InputStateGlobal import inputState
from direct.actor.Actor import Actor

from panda3d.core import ClockObject
from panda3d.core import AmbientLight
from panda3d.core import DirectionalLight
from panda3d.core import Vec3
from panda3d.core import Vec4
from panda3d.core import Point3
from panda3d.core import TransformState
from panda3d.core import BitMask32

from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletPlaneShape
from panda3d.bullet import BulletBoxShape
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletDebugNode
from panda3d.bullet import BulletVehicle
from panda3d.bullet import ZUp


class Game(DirectObject):
    def __init__(self):
        base.setBackgroundColor(0.1, 0.1, 0.8, 1)
        base.setFrameRateMeter(True)

        base.cam.setPos(0, -20, 4)
        base.cam.lookAt(0, 0, 0)

        # Light
        alight = AmbientLight('ambientLight')
        alight.setColor(Vec4(0.5, 0.5, 0.5, 1))
        alightNP = base.render.attachNewNode(alight)

        dlight = DirectionalLight('directionalLight')
        dlight.setDirection(Vec3(1, 1, -1))
        dlight.setColor(Vec4(0.7, 0.7, 0.7, 1))
        dlightNP = base.render.attachNewNode(dlight)

        base.render.clearLight()
        base.render.setLight(alightNP)
        base.render.setLight(dlightNP)

        # Input
        self.accept('escape', self.doExit)
        self.accept('r', self.doReset)
        self.accept('f1', self.toggleWireframe)
        self.accept('f2', self.toggleTexture)
        self.accept('f3', self.toggleDebug)
        self.accept('f5', self.doScreenshot)

        inputState.watchWithModifiers('forward', 'w')
        inputState.watchWithModifiers('left', 'a')
        inputState.watchWithModifiers('reverse', 's')
        inputState.watchWithModifiers('right', 'd')
        inputState.watchWithModifiers('turnLeft', 'q')
        inputState.watchWithModifiers('turnRight', 'e')

        # Task
        base.taskMgr.add(self.update, 'updateWorld')

        # Physics
        self.setup()

    # _____HANDLER_____

    def doExit(self):
        self.cleanup()
        sys.exit(1)

    def doReset(self):
        self.cleanup()
        self.setup()

    def toggleWireframe(self):
        base.toggleWireframe()

    def toggleTexture(self):
        base.toggleTexture()

    def toggleDebug(self):
        if self.debugNP.isHidden():
            self.debugNP.show()
        else:
            self.debugNP.hide()

    def doScreenshot(self):
        base.screenshot('Bullet')

    # ____TASK___

    def processInput(self, dt):
        engineForce = 0.0
        brakeForce = 0.0

        if inputState.isSet('forward'):
            engineForce = self.car_mass * 1.5
            brakeForce = 0.0

        if inputState.isSet('reverse'):
            engineForce = 0.0
            brakeForce = self.car_mass * 0.5

        if inputState.isSet('turnLeft'):
            self.steering += dt * self.steeringIncrement
            self.steering = min(self.steering, self.steeringClamp)

        if inputState.isSet('turnRight'):
            self.steering -= dt * self.steeringIncrement
            self.steering = max(self.steering, -self.steeringClamp)

        # Apply steering to front wheels
        self.vehicle.setSteeringValue(self.steering, 0)
        self.vehicle.setSteeringValue(self.steering, 1)

        # Apply engine and brake to rear wheels
        self.vehicle.applyEngineForce(engineForce, 2)
        self.vehicle.applyEngineForce(engineForce, 3)
        self.vehicle.setBrake(brakeForce, 2)
        self.vehicle.setBrake(brakeForce, 3)

    def update(self, task):
        globalClock = ClockObject.get_global_clock()
        dt = globalClock.getDt()

        self.processInput(dt)
        self.world.doPhysics(dt, 10, 0.008)

        # print(self.vehicle.getWheel(0).getRaycastInfo().isInContact())
        # print(self.vehicle.getWheel(0).getRaycastInfo().getContactPointWs())
        # print(self.vehicle.getChassis().isKinematic())

        return task.cont

    def cleanup(self):
        self.world = None
        self.worldNP.removeNode()

    def setup(self):
        self.worldNP = base.render.attachNewNode('World')

        # World
        self.debugNP = self.worldNP.attachNewNode(BulletDebugNode('Debug'))
        self.debugNP.show()

        self.world = BulletWorld()
        self.world.setGravity(Vec3(0, 0, -9.81))
        self.world.setDebugNode(self.debugNP.node())

        # Plane
        shape = BulletPlaneShape(Vec3(0, 0, 1), 0)

        np = self.worldNP.attachNewNode(BulletRigidBodyNode('Ground'))
        np.node().addShape(shape)
        np.setPos(0, 0, 0)
        np.setCollideMask(BitMask32.allOn())

        # planeNP = base.loader.loadModel('models/CityEnv/plane.egg')
        # planeNP.setPos(0,0,-1)
        # planeTex = base.loader.loadTexture('models/CityTerrain/surface.tif')
        # planeNP.setTexture(planeTex)
        # planeNP.reparentTo(np)

        self.world.attachRigidBody(np.node())

        # Chassis
        self.car_length = 4.775
        self.car_width = 1.800
        self.car_hight = 1.435
        self.car_mass = 1685.0
        shape = BulletBoxShape(Vec3(self.car_width+0.2, self.car_length+0.2, self.car_hight))
        ts = TransformState.makePos(Point3(0, 0, self.car_hight))

        np = self.worldNP.attachNewNode(BulletRigidBodyNode('Vehicle'))
        np.node().addShape(shape, ts)
        np.setPos(0, 0, 0.5)
        np.node().setMass(self.car_mass)
        np.node().setDeactivationEnabled(False)

        self.world.attachRigidBody(np.node())

        # np.node().setCcdSweptSphereRadius(1.0)
        # np.node().setCcdMotionThreshold(1e-7)

        # Vehicle
        self.vehicle = BulletVehicle(self.world, np.node())
        self.vehicle.setCoordinateSystem(ZUp)
        self.world.attachVehicle(self.vehicle)

        self.vehBody = base.loader.loadModel('models/sedan/body.egg')
        # self.vehBody = Actor('models/sedan/body_act.egg')
        self.vehBody.setScale(0.85, 0.85, 0.85)
        # bodyTex = base.loader.loadTexture('models/sedan/body.tif')
        # self.yugoNP.setTexture(bodyTex)
        # self.yugoNP.setHpr(0, 0, 0)
        self.vehBody.setPos(0, 0, -self.car_hight/2)
        self.vehBody.reparentTo(np)

        # Set Tire
        tire_rad = 0.5
        axle_hight = tire_rad - 0.2
        tireTex = base.loader.loadTexture('models/sedan/tire.png')

        # Right front wheel
        wheelrfNp = base.loader.loadModel('models/sedan/tireR.egg')
        wheelrfNp.setTexture(tireTex)
        wheelrfNp.reparentTo(self.worldNP)
        self.addWheel(Point3(self.car_width / 2 + 0.5, self.car_length / 2 + 0.6, axle_hight),
                      True, wheelrfNp,
                      tire_rad)

        # Left front wheel
        wheellfNp = base.loader.loadModel('models/sedan/tireL.egg')
        wheellfNp.setTexture(tireTex)
        wheellfNp.reparentTo(self.worldNP)
        self.addWheel(Point3(-(self.car_width / 2 + 0.5), self.car_length / 2 + 0.6, axle_hight),
                      True, wheellfNp,
                      tire_rad)

        # Right rear wheel
        wheelrrNp = base.loader.loadModel('models/sedan/tireR.egg')
        wheelrrNp.setTexture(tireTex)
        wheelrrNp.reparentTo(self.worldNP)
        self.addWheel(Point3(self.car_width / 2 + 0.5, -(self.car_length / 2 + 0.3), axle_hight),
                      True, wheelrrNp,
                      tire_rad)

        # Left rear wheel
        wheellrNp = base.loader.loadModel('models/sedan/tireR.egg')
        wheellrNp.setTexture(tireTex)
        wheellrNp.reparentTo(self.worldNP)
        self.addWheel(Point3(-(self.car_width / 2 + 0.5), -(self.car_length / 2 + 0.3), axle_hight),
                      True, wheellrNp,
                      tire_rad)

        # Steering info
        self.steering = 0.0  # degree
        self.steeringClamp = 45.0  # degree
        self.steeringIncrement = 120.0  # degree per second

    def addWheel(self, pos, isfront, np, rad):
        wheel = self.vehicle.createWheel()

        wheel.setNode(np.node())
        wheel.setChassisConnectionPointCs(pos)
        wheel.setFrontWheel(isfront)

        wheel.setWheelDirectionCs(Vec3(0, 0, -1))
        wheel.setWheelAxleCs(Vec3(1, 0, 0))
        wheel.setWheelRadius(rad)
        wheel.setMaxSuspensionTravelCm(40.0)

        wheel.setSuspensionStiffness(40.0)
        wheel.setWheelsDampingRelaxation(2.3)
        wheel.setWheelsDampingCompression(4.4)
        wheel.setFrictionSlip(self.car_mass * 0.1)
        wheel.setRollInfluence(0.1)


game = Game()
base.run()
