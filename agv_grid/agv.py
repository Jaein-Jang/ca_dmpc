class AGV:
    def __init__(self, agv_id, row, column, frames):
        self.id = agv_id
        self.row = row
        self.column = column

        self.path = []
        self.travel_distance = 0.

        #for animation
        self.target = None
        self.progress = 0.0
        self.speed = 1.0/frames

    def set_path(self, path):
        self.path = path


    def move(self, direction):
        if direction == "up":
            self.row -= 1
        elif direction == "down":
            self.row += 1
        elif direction == "left":
            self.column -= 1
        elif direction == "right":
            self.column += 1

    def start_next_step(self):
        if not self.path:
            #no path set or finished
            return False

        self.target = self.path.pop(0)
        self.progress = 0.0
        return True

    def has_next(self):
        return len(self.path) > 0

    def follow_path(self):
        if self.path:
            self.row, self.column = self.path.pop(0)

    def update(self):
        if self.target is None:
            return False

        self.progress += self.speed
        if self.progress >= 1.0:
            self.row, self.column = self.target
            self.target = None
            return True

        return False
