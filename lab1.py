"""
Sample Solution for Lab1
Use "run.py [--sim] lab1_solution" to execute
"""


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()

    def forward(self):
        self.create.drive_direct(100, 100)
        self.time.sleep(3)

    def backward(self):
        self.create.drive_direct(-100, -100)
        self.time.sleep(3)

    def stop(self):
        self.create.drive_direct(0, 0)

    def left_in_place(self):
        self.create.drive_direct(100, -100)
        self.time.sleep(3)

    def right_in_place(self):
        self.create.drive_direct(-100, 100)
        self.time.sleep(3)

    def left_and_forward(self):
        self.create.drive_direct(100, 50)
        self.time.sleep(6)

    def run(self):
        self.create.start()
        self.create.safe()

        self.forward()
        self.backward()
        self.left_in_place()
        self.right_in_place()
        self.left_and_forward()
        self.stop()

        self.create.stop()
