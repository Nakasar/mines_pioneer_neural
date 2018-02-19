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

        self.alpha = [1/4, 1/4, 1/(math.pi)]
        self.sensor_alpha = 1/4

    def train(self, target):
        position = self.robot.get_position()
        sensors = self.robot.get_sensors_distances();

        network_input = [0] * 11
        network_input[0] = (position[0]-target[0])
        network_input[1] = (position[1]-target[1])
        network_input[2] = (position[2]-target[2])
        for i in range(len(sensors)) :
            network_input[i+3] = sensors[i] * 1

        while self.running:
            debut = time.time()
            command = self.network.runNN(network_input)
            self.robot.set_motor_velocity(command)

            time.sleep(0.050)
            position = self.robot.get_position()
            sensors = self.robot.get_sensors_distances()

            network_input[2]=(position[2]-target[2]-theta_s(position[0], position[1]))*self.alpha[2]
            network_input[1] = (position[1]-target[1])*self.alpha[1]
            network_input[0] = (position[0]-target[0])*self.alpha[0]
            for i in range(len(sensors)) :
                network_input[i+3] = sensors[i] * self.sensor_alpha

            if self.training:
                delta_t = (time.time()-debut)
                sensors_influence_rd = 0
                sensors_influence_rg = 0
                # ce dictionnaire contient les angles des capteurs (utile pour obtenir leur influence)
                sensors_angle = {0: -math.pi / 6, 1: -math.pi/3, 2: -2*math.pi/3, 3: -5*math.pi/6, 4: 5*math.pi/6, 5: 2*math.pi/3, 6: math.pi/3, 7: math.pi/6}
                # TODO: Modify grad for proper retro-propagation
                for k in range(len(sensors)):
                    offset = network_input[3 + k] * delta_t * self.robot.r / self.robot.R
                    if k == 0 or k == 7:
                        vects = [1, -1]
                    elif k == 1 or k == 2:
                        vects = [1, 1]
                    elif k == 3 or k == 4:
                        vects = [-1, 1]
                    else:
                        vects = [-1, -1]
                    sensors_influence_rd += vects[0] * offset
                    sensors_influence_rg += vects[1] * offset

                grad = [
                    ((-1)/(delta_t**2))*(network_input[0]*delta_t*self.robot.r*math.cos(position[2])
                    +network_input[1]*delta_t*self.robot.r*math.sin(position[2])
                    -network_input[2]*delta_t*self.robot.r/(2*self.robot.R))
                    +sensors_influence_rg,

                    ((-1)/(delta_t**2))*(network_input[0]*delta_t*self.robot.r*math.cos(position[2])
                    +network_input[1]*delta_t*self.robot.r*math.sin(position[2])
                    +network_input[2]*delta_t*self.robot.r/(2*self.robot.R))
                    +sensors_influence_rd
                ]

                # The two args after grad are the gradient learning steps for t
                # and t-1
                self.network.backPropagate(grad, 0.05, 0)

        self.robot.set_motor_velocity([0,0])
        self.running = False
