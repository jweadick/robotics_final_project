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
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        goal_x = 0.5
        goal_y = -0.5
        base_speed = 100

        result = np.empty((0,5))
        end_time = self.time.time() + 10
        while self.time.time() < end_time:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
                new_row = [self.time.time(), math.degrees(self.odometry.theta), math.degrees(goal_theta), self.odometry.x, self.odometry.y]
                result = np.vstack([result, new_row])

                output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                # base version:
                # self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))

                # improved version 1: stop if close enough to goal
                # distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                # if distance < 0.1:
                #     break

                # improved version 2: fuse with velocity controller
                distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                output_distance = self.pidDistance.update(0, distance, self.time.time())
                self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))

        plt.figure()
        plt.plot(result[:,0], result[:,1])
        plt.plot(result[:,0], result[:,2])
        plt.savefig("angle.png")

        plt.figure()
        plt.plot(result[:,3], result[:,4])
        plt.savefig("position.png")
