from pyCreate2 import create2
import math
import numpy as np
import odometry
import pd_controller2
import pid_controller
import matplotlib
# if on the robot, don't use X backend
matplotlib.use('Agg')
import matplotlib.pyplot as plt


class Run:
    def __init__(self, factory):
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.sonar = factory.create_sonar()
        self.servo = factory.create_servo()
        self.odometry = odometry.Odometry()
        # self.pidTheta = pd_controller2.PDController(500, 100, -200, 200, is_angle=True)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        goal_theta = -math.pi / 2.0

        result = np.empty((0,3))
        end_time = self.time.time() + 10
        while self.time.time() < end_time:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                new_row = [self.time.time(), math.degrees(self.odometry.theta), math.degrees(goal_theta)]
                result = np.vstack([result, new_row])

                output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
                self.create.drive_direct(int(output_theta), int(-output_theta))

        plt.plot(result[:,0], result[:,1])
        plt.plot(result[:,0], result[:,2])
        plt.savefig("plot.png")
