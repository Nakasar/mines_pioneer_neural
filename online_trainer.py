import time
import math


def theta_s(x,y):
    if x>0:
        return math.atan(10*y)
    if x<=0:
        return math.atan(-10*y)

class OnlineTrainer:
    def __init__(self, robot, NN):
        """
        Args:
            robot (Robot): a robot instance following the pattern of
                VrepPioneerSimulation
            target (list): the target position [x,y,theta]
        """
        self.robot = robot
        self.network = NN

        self.alpha = [1/4, 1/4, 1/(math.pi), 1/4, 1/4, 1/4, 1/4]

    def train(self, target):
        position = self.robot.get_position()
        sensors = self.robot.get_sensors_distance()

        network_input = [0] * 7
        network_input[0] = (position[0]-target[0])
        network_input[1] = (position[1]-target[1])
        network_input[2] = (position[2]-target[2])
        network_input[3] = (sensors[0])
        network_input[4] = (sensors[1])
        network_input[5] = (sensors[2])
        network_input[6] = (sensors[3])

        while self.running:
            debut = time.time()
            command = self.network.runNN(network_input)
            self.robot.set_motor_velocity(command)
            time.sleep(0.050)
            position = self.robot.get_position()
            sensors = self.robot.get_sensors_distance()
            network_input[2]=(position[2]-target[2]-theta_s(position[0], position[1]))*self.alpha[2]
            network_input[1] = (position[1]-target[1])*self.alpha[1]
            network_input[0] = (position[0]-target[0])*self.alpha[0]
            network_input[3] = (sensors[0]) * self.alpha[3]
            network_input[4] = (sensors[1]) * self.alpha[4]
            network_input[5] = (sensors[2]) * self.alpha[5]
            network_input[6] = (sensors[3]) * self.alpha[6]
            if self.training:
                delta_t = (time.time()-debut)
                # TODO: Modify grad for proper retro-propagation
                grad = [
                    ((-1)/(delta_t**2))*(network_input[0]*delta_t*self.robot.r*math.cos(position[2])
                    +network_input[1]*delta_t*self.robot.r*math.sin(position[2])
                    -network_input[2]*delta_t*self.robot.r/(2*self.robot.R)),

                    ((-1)/(delta_t**2))*(network_input[0]*delta_t*self.robot.r*math.cos(position[2])
                    +network_input[1]*delta_t*self.robot.r*math.sin(position[2])
                    +network_input[2]*delta_t*self.robot.r/(2*self.robot.R))
                ]

                # The two args after grad are the gradient learning steps for t
                # and t-1
                self.network.backPropagate(grad, 0.05, 0)

        self.robot.set_motor_velocity([0,0])
        self.running = False
