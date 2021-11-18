"""
Sample Solution for Lab3
Use "run.py [--sim] lab3_solution" to execute
"""

from pyCreate2 import create2
import math
import odometry
import matplotlib.pyplot as plt

groundTruthX = []
groundTruthY = []
odX = []
odY = []

class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.odometry = odometry.Odometry()
        
    def sleep(self, time_in_sec):
        start = self.time.time()
        last_update = None
        while True:
            state = self.create.update()
            t = self.time.time()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # don't include the groundTruth data if you run it on the robot!
                groundTruth = self.create.sim_get_position()
                groundTruthX.append(groundTruth[0]-0.58)
                groundTruthY.append(groundTruth[1]+0.17)
                odX.append(self.odometry.x)
                odY.append(self.odometry.y)
                print("{},{},{},{},{}".format(self.odometry.x, self.odometry.y, self.odometry.theta * 180 / math.pi, groundTruth[0], groundTruth[1]))
            if start + time_in_sec <= t:
                break

    def turn_left(self, speed, angle):
        self.create.drive_direct(speed, -speed)
        self.sleep(math.pi * create2.Specs.WheelDistanceInMM / (360 / angle) / speed)

    def forward(self, speed, distance):
        self.create.drive_direct(speed, speed)
        self.sleep(distance / speed)

    def rectangle(self, speed):
        self.forward(speed, 1000)
        self.turn_left(speed, 90)
        self.forward(speed, 500)
        self.turn_left(speed, 95)
        self.forward(speed, 1000)
        self.turn_left(speed, 95)
        self.forward(speed, 500)
        self.turn_left(speed, 90)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        # normal speed
        self.rectangle(100)

        # high speed
        #self.rectangle(300)
        
        # Move forward
        #self.create.drive_direct(100, 100)
        #self.sleep(5)

        plt.plot(groundTruthX, groundTruthY, label='Ground Truth')
        plt.plot(odX, odY, label='Odometry')
        plt.legend(loc='upper left')
        plt.show()
