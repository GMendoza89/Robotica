import coppeliaRobots as CP
import time

if __name__ == "__main__":
    robot = CP.PioneerP3DX(0.1,0.5)
    robot.connect(19999)
    robot.robotInit()
    while True:
        robot.move(0.1,0.0)
        time.sleep(12)
        robot.move(0.0,-0.1)
        time.sleep(2.1)