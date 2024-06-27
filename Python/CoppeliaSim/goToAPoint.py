import coppeliaRobots as CP

if __name__ == "__main__":
    robot = CP.PioneerP3DX(0.1,0.5)
    robot.connect(19999)
    robot.robotInit()
    robot.goToAPoint([0,0,0])