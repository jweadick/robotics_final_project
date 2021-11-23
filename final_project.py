from pyCreate2 import create2
import math
import odometry
import pid_controller
import lab8_map
import rrt_map
import particle_filter
import rrt
import numpy as np
import pdb



class Run:
    def __init__(self, factory):
        """Constructor.
        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        self.arm = factory.create_kuka_lbr4p()
        self.virtual_create = factory.create_virtual_create()
        # self.virtual_create = factory.create_virtual_create("192.168.1.XXX")
        self.odometry = odometry.Odometry()
        self.mapJ = lab8_map.Map("lab8_map.json")
        self.map = rrt_map.Map("configuration_space.png")
        self.rrt = rrt.RRT(self.map)

        # TODO identify good PID controller gains
        self.pidTheta = pid_controller.PIDController(200, 0, 100, [-10, 10], [-50, 50], is_angle=True)
        # TODO identify good particle filter parameters
        self.pf = particle_filter.ParticleFilter(self.mapJ, 1000, 0.06, 0.15, 0.2)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)
        self.joint_angles = np.zeros(7)

    def sleep(self, time_in_sec):
        """Sleeps for the specified amount of time while keeping odometry up-to-date
        Args:
            time_in_sec (float): time to sleep in seconds
        """
        start = self.time.time()
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))
            t = self.time.time()
            if start + time_in_sec <= t:
                break

    def go_to_angle(self, goal_theta):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.fabs(math.atan2(
            math.sin(goal_theta - self.odometry.theta),
            math.cos(goal_theta - self.odometry.theta))) > 0.05:
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(+output_theta), int(-output_theta))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def forward(self):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        base_speed = 100
        distance = 0.5
        goal_x = self.odometry.x + math.cos(self.odometry.theta) * distance
        goal_y = self.odometry.y + math.sin(self.odometry.theta) * distance
        while True:
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))

            # stop if close enough to goal
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            if distance < 0.05:
                self.create.drive_direct(0, 0)
                break
            self.sleep(0.01)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)

    def go_to_goal(self, goal_x, goal_y):
        old_x = self.odometry.x
        old_y = self.odometry.y
        old_theta = self.odometry.theta
        while math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2)) > 0.1:
            goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            output_distance = self.pidDistance.update(0, distance, self.time.time())
            self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))
            self.sleep(0.01)
        self.create.drive_direct(0, 0)
        self.pf.move_by(self.odometry.x - old_x, self.odometry.y - old_y, self.odometry.theta - old_theta)


    def take_measurements(self):
        angle = -90
        while angle <= 90:
            self.servo.go_to(angle)
            self.time.sleep(2.0)
            distance = self.sonar.get_distance()
            #print(distance)
            self.pf.measure(distance, math.radians(angle))
            #self.pf._map.draw(self.pf, "test{}.png".format(self._pos))
            x, y, theta = self.pf.get_estimate()
            self.virtual_create.set_pose((x, y, 0.1), theta)
    
            data = []
            for particle in self.pf._particles:
                data.extend([particle.x, particle.y, 0.1, particle.theta])
    
            angle += 45
            #self._pos += 1

    def visualize(self):
        x, y, theta = self.pf.get_estimate()
        self.virtual_create.set_pose((x, y, 0.1), theta)
        data = []
        for particle in self.pf._particles:
            data.extend([particle.x, particle.y, 0.1, particle.theta])
        self.virtual_create.set_point_cloud(data)

    def run(self):
        
        #print(self.create.sim_get_position())

        startCoords = self.create.sim_get_position()
        start_x = startCoords[0]*100
        start_y = 300 - startCoords[1]*100

        self.rrt.build((100, 250), 3000, 10)
        x_goal = self.rrt.nearest_neighbor((150, 30))
        path = self.rrt.shortest_path(x_goal)

        for v in self.rrt.T:
            for u in v.neighbors:
                self.map.draw_line((v.state[0], v.state[1]), (u.state[0], u.state[1]), (0,0,0))
        for idx in range(0, len(path)-1):
            self.map.draw_line((path[idx].state[0], path[idx].state[1]), (path[idx+1].state[0], path[idx+1].state[1]), (0,255,0))

        self.map.save("final_project_rrt.png")
        path = self.rrt.shortest_path(x_goal)

        self.create.start()
        self.create.safe()

        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        # lab 10 solution to get the robot to move
        self.odometry.x = 1
        self.odometry.y = 0.5
        self.odometry.theta = 0
        base_speed = 75

        print("number of coords:", len(path))
        num_points = 0
        self.take_measurements()
        for p in path:
            #print(p.state)
            goal_x = p.state[0] / 100.0
            goal_y = 3.0 - p.state[1] / 100.0
            print("goal x y: ", goal_x, goal_y)
             # taken from lab 8&9 solution
            self.visualize()
            # self.pf._pos = 0
            if num_points == 3:
                num_points = 0
                self.create.drive_direct(0, 0)
                self.take_measurements()
            else:
                num_points += 1
            self.go_to_goal(goal_x, goal_y)
            # self.take_measurements()
            # while True:
            #     state = self.create.update()
            #     if state is not None:
            #         self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
            #         goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
            #         theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
            #         output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())
            #         self.create.drive_direct(int(base_speed+output_theta), int(base_speed-output_theta))
            #         # print("[{},{},{}]".format(self.odometry.x, self.odometry.y, math.degrees(self.odometry.theta)))

            #         distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
            #         if distance < 0.05:
            #             break



        #self.create.drive_direct(0, 0)

        
        #100 250

        #self.arm.open_gripper()
        #self.time.sleep(4)
        #self.arm.close_gripper()

        # request sensors
        '''
        self.visualize()
        self.virtual_create.enable_buttons()
        self.visualize()'''

		
        ''' 
        self.arm.go_to(4, math.radians(-90))
        self.time.sleep(4)

        while True:
            b = self.virtual_create.get_last_button()
            if b == self.virtual_create.Button.MoveForward:
                self.forward()
                self.visualize()
            elif b == self.virtual_create.Button.TurnLeft:
                self.go_to_angle(self.odometry.theta + math.pi / 2)
                self.visualize()
            elif b == self.virtual_create.Button.TurnRight:
                self.go_to_angle(self.odometry.theta - math.pi / 2)
                self.visualize()
            elif b == self.virtual_create.Button.Sense:
                distance = self.sonar.get_distance()
                print(distance)
                self.pf.measure(distance, 0)
                self.visualize()
				
            posC = self.create.sim_get_position()
			
            print(posC)
			
            self.arm.go_to(4, math.radians(-90))
            self.arm.go_to(5, math.radians(90))
            #self.time.sleep(100)
            self.time.sleep(0.01)
            '''
            