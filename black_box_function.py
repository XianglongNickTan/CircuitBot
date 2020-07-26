import time

class MoveItIkDemo:
    def __init__(self):
        self.x = 0
        self.y = 0

    def black_box_function(self):
        return self.x + self.y

    def draw_circle(self, next_to_probe):
        self.x = next_to_probe['x']
        self.y = next_to_probe['y']
        voltage = self.black_box_function()

        time.sleep(1)

        voltage_file = open("voltage.txt", "w")
        voltage_file.write(str(voltage))
        voltage_file.close()
