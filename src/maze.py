import sys
import random
from tkinter import *

class Maze(object):
    def __init__(self, width=35, height=21, recursion_limit=2000):
        self.check_size(width, height)
        self.maze = dict()
        try:
            sys.setrecursionlimit(recursion_limit)
        except Exception as e:
            print(e)
 
    def check_size(self, width, height):
        # width and height must be odd
        if width % 2 == 0:
            self.width = width + 1
        else:
            self.width = width

        if height % 2 == 0:
            self.height = height + 1
        else:
            self.height = height

    def init_maze(self):
        for x in range(0, self.width):
            self.maze[x] = dict()
            for y in range(0, self.height):
                self.maze[x][y] = 1

    def carve_maze(self, x, y):
        dir = random.randint(0, 3)
        count = 0
        while count < 4:
            dx = 0
            dy = 0
            if dir == 0:
               dx = 1
            elif dir == 1:
               dy = 1
            elif dir == 2:
               dx = -1
            else:
               dy = -1
            x1 = x + dx
            y1 = y + dy
            x2 = x1 + dx
            y2 = y1 + dy
            if x2 > 0 and x2 < self.width and y2 > 0 and y2 < self.height:
               if self.maze[x1][y1] == 1 and self.maze[x2][y2] == 1:
                  self.maze[x1][y1] = 0
                  self.maze[x2][y2] = 0
                  self.carve_maze(x2, y2)
            count = count + 1
            dir = (dir + 1) % 4

    def generate_maze(self):
        random.seed()
        self.maze[1][1] = 0
        self.carve_maze(1, 1)
        self.maze[1][0] = 0
        self.maze[self.width - 2][self.height - 1] = 0
    
    def init(self):
        self.init_maze()
        self.generate_maze()
        self.maze[1][0] = ' @ '
        self.maze[self.height-2][self.width-1] = ' * '

    def get_maze(self):
        return self.maze
