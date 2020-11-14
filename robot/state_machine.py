from threading import Thread

class StateMachine:

    def __init__(self):
        self.state = 0
        self.state_desc = [
            'init',
            'request target',
            'face target',
            'drive to target',
            'reach target'
        ]
        self.state_max = len(self.state_desc)
        self.is_running = False
        self.pos = [0, 0]
        self.dst = None
        self.dir = 0

        self.thread = None

    def setup(self):
        pass

    def run(self):
        
        while self.is_running:

            # reads data
            
            # action
            if self.state == 0:
                print('ROB >> Initial State')

            elif self.state == 1:

                while self.dst is None:
                    try:
                        cmd_input = input('BOT >> request target: ').strip()
                        if cmd_input == 'exit':
                            self.is_running = False
                            break
                        req_dst = [float(x) for x in cmd_input.split()]
                        if len(req_dst) == 2:
                            self.dst = req_dst
                    except:
                        print('BOT >> ERR: Invalid target!')

                if not self.is_running:
                    break

                print('BOT >> target =', self.dst)

            elif self.state == 2:

                pass

            elif self.state == 3:

                pass

            elif self.state == 4:

                pass


            # sets speed

            # changes state
            if self.state == 0:
                self.state = 1
            elif self.state == 1:
                pass
            elif self.state == 2:
                pass
            elif self.state == 3:
                pass
            elif self.state == 4:
                pass


    def start(self):
        print('BOT >> Robot start.')
        if not self.is_running:
            self.thread = Thread(target=self.run)
            self.is_running = True
            self.thread.start()
        else:
            print('BOT >> ERR: Attempt to start Robot while it\'s running!')

    def stop(self):
        print('BOT >> Attempt to stop Robot.')
        self.is_running = False
        print('BOT >> Robot stopped.')