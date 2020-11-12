from robot.robot import Robot
from acuro.acuro import Acuro
from acuro.display import Display

def main():
    # acuro = Acuro()
    robot = Robot()

    display = Display()
    display.add('Video')

    print(display.windows)

    # acuro.start()
    robot.start()

    robot.thread.join()

if  __name__ == '__main__':
    main()