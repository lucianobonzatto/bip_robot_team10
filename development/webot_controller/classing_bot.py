from controller import Robot
from robot import My_Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# distance sensors
ps = []
psNames = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(timestep)
    
#ground sensor
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDevice(gsNames[i]))
    gs[i].enable(timestep)
    
# motors    
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# encoders
encoder = []
encoderNames = ['left wheel sensor', 'right wheel sensor']
for i in range(2):
    encoder.append(robot.getDevice(encoderNames[i]))
    encoder[i].enable(timestep)

delta_t = robot.getBasicTimeStep()/1000.0
R = 0.0205
D = 0.0565
# A = 0.05

my_robot = My_Robot(gs, leftMotor, rightMotor, encoder, delta_t, R, D)

while robot.step(timestep) != -1:
    my_robot.update()
    my_robot.action()
    pass
