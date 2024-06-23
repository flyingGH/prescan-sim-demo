import json


class MooreStateMachine:
    def __init__(self, file_path):

        with open(file_path, 'r') as file:
            info = json.load(file)

        self.states = info['states']
        self.current_state = 'Sinit'

        self.transition_table = info['transition']
        self.output_table = info['output']


    def reaction(self, warning, urgent):

        if warning & urgent:
            input = 'X'
        elif warning & (not urgent):
            input = 'W'
        elif (not warning) & urgent:
            input = 'U'
        else:
            input = 'O'

        self.current_state = self.transition_table[str(self.current_state)][input]


    def get_output(self):
        return self.output_table[str(self.current_state)]

    def process_inputs(self, inputs):
        for input in inputs:
            self.transition(input)
            print("Current state:", self.current_state, "Output:", self.get_output())


# Example usage
if __name__ == "__main__":
    sm = MooreStateMachine()

    n = 0
    WARNING = False
    URGENT = False

    while n < 50:
        
        if (n > 10) & (n <20):
            WARNING = True
        else:
            WARNING = False
        print("URGENT is " + str(URGENT) + " and WARNING is " + str(WARNING) + " at time" + str(n))
        sm.reaction(WARNING, URGENT)
        out = sm.get_output()
        print(out)
        
        n += 1
