import turtle
import time
from warehouse import Warehouse

class Render:
    def __init__(self, _map, cell_size):
        self._map = _map
        self.rows = _map.shape[0]
        self.columns = _map.shape[1]
        self.cell_size = cell_size

        #store AGV turtles
        self.agvs = {}

        #seperate turtle for AGV id labeling
        self.agv_labels = {}

        #set screen
        self.screen = turtle.Screen()
        self.screen.setup(width=self.columns*self.cell_size + 50,
                          height=self.rows*self.cell_size + 50)
        self.screen.tracer(0)

        #set drawer(turtle)
        self.drawer = turtle.Turtle()
        self.drawer.hideturtle()
        self.drawer.penup()
        self.drawer.speed(0)


    def grid_to_xy(self, row, col):
        x = col*self.cell_size - (self.columns*self.cell_size)/2
        y = (self.rows*self.cell_size)/2 - row*self.cell_size
        return x, y


    def draw_cell(self, x, y, fill=False):
        self.drawer.penup()
        self.drawer.goto(x, y)
        self.drawer.setheading(0)
        self.drawer.pendown()

        if fill:
            self.drawer.begin_fill()

        for _ in range(4):
            self.drawer.forward(self.cell_size)
            self.drawer.right(90)

        if fill:
            self.drawer.end_fill()

        self.drawer.penup()

    def map(self):
        for r in range(self.rows):
            for c in range(self.columns):
                x, y = self.grid_to_xy(r, c)

                if self._map[r][c] == 1:
                    self.drawer.fillcolor('gray')
                    self.draw_cell(x, y, fill=True)
                else:
                    self.draw_cell(x, y, fill=False)

        self.screen.update()


    def add_agv(self, agv):
        body = turtle.Turtle()
        body.shape('square')
        body.color('blue')
        body.penup()
        body.speed(0)

        label = turtle.Turtle()
        label.hideturtle()
        label.penup()
        label.speed(0)
        label.pencolor('black')

        self.agvs[agv.id] = body
        self.agv_labels[agv.id] = label
        
        self.move(agv) #draw agv after init


    def move(self, agv): #, steps, delay):
        body = self.agvs[agv.id]
        label = self.agv_labels[agv.id]

        if agv.target:
            r0, c0 = agv.row, agv.column
            r1, c1 = agv.target
            t = agv.progress

            r = r0 + (r1 - r0)*t
            c = c0 + (c1 - c0)*t

        else:
            r, c = agv.row, agv.column

        x, y =self.grid_to_xy(r, c)
        x += self.cell_size/2
        y -= self.cell_size/2
        body.goto(x, y)

        label.goto(x, y - self.cell_size)
        label.clear()
        label.write(str(agv.id), align='center', font=('Arial', 8, 'bold'))


    def draw_path(self, path, color="red"):
        t = turtle.Turtle()
        t.hideturtle()
        t.penup()
        t.speed(0)
        t.pencolor(color)
        t.width(2)

        first = True
        for r, c in path:
            x, y = self.grid_to_xy(r, c)
            x += self.cell_size / 2
            y -= self.cell_size / 2
            
            if first:
                t.goto(x, y)
                t.pendown()
                first = False
            else:
                t.goto(x, y)

        self.screen.update()
