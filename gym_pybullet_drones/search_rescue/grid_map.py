import random
import numpy as np
from utils import addPoints
from constants import *

from copy import deepcopy

class CellState:
    '''
        States of the cell of the grid
        represent the probability of the target being found in this cell
    '''
    HIGH   =    67.0    # from 67%-99% - red      [255,0,0]
    MEDIUM =   34.0    # from 34%-66% - yellow 	(255,215,0)
    LOW    =    1.0     # from 1%-33%  - blue     (100,149,237)
    FOUND  =   100.0   # target found - green    (0,128,0)   
    VISITED = 0.0


    def get_random_state():
        states = [CellState.HIGH, CellState.MEDIUM, CellState.LOW]
        return random.choice(states)

NOT_VISITED = 0      
VISITED = 1
OBSTACLE = 2

DRAW_CELLS = False

RED    = [1,0,0]
YELLOW = [1,1,0]
GRENN  = [0,1,0]
BLUEE  = [0,0,1]
GRAY   = [.7,.7,.7]

class GridMap(object):
    def __init__(self, rows, cols, pybullet) -> None:
        self.pybullet = pybullet
        self.rows = rows
        self.cols = cols 
        self.cells =  np.ndarray((self.rows+1,self.cols+1), dtype=Cell) # grid memory using numpy array
        
        # parameters
        self.z_offset = 0.1

        self.create_grid_cells()  
        self.person = Person(self, pybullet) # Where the person is located
        self.person.draw()

        
    
        print(f' -----> Grid created with  col:{self.cols} row:{self.rows}')

    def create_grid_cells(self):
        '''
            Creates grid with cells according to the number of rows and cols
        '''
        size_block = 1
        z = self.z_offset
        for y in range(0, self.rows, size_block ):
            for x in range(0, self.cols, size_block ):
                #           row     col       
                self.cells[int(x)][int(y)] = Cell(self.pybullet, [x,y,z], CellState.get_random_state())

    def draw_map(self):
        size_block = 1
        for y in range(0, self.rows, size_block ):
            for x in range(0, self.cols, size_block ):
                # draw a cell
                self.cells[y][x].draw_cell() 

    def get_random_cell(self):
        x = random.randint(0, self.rows - 1)
        y = random.randint(0, self.cols - 1)
        cell = self.cells[int(x)][int(y)]
        center = cell.get_center() 
        center[2] += OFFSET_ALTITUDE # default altitude 

        return center

    def get_cell(self, position):
        return self.cells[int(position[0])][int(position[1])]

    def update_state(self, row, col):
        pass

    def get_sucessors(self, cell):
        """
            Obtains a list of the 8-connected successors of the node at (i, j).

            :param cell: position cell .
            :type tuple: int.
           
            :return: list of the 8-connected successors.
            :rtype: list of cells.
        """
        i = cell[0]
        j = cell[1]
        successors = []
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if (dx,dy)!= (0,0):
                    x = i + dx
                    y = j + dy
                    # if not visited
                    if x >= 0 and y >= 0:
                        if self.get_state_cell([x,y]) == NOT_VISITED:
                            successors.append(self.get_cell_center([x,y]))  
                            #successors.append((x, y))
        
        #print(successors)
        #input()
        return successors

    def get_state_cell(self, cell):
        '''
            Get if cell was visisted before
            cell: tuple with coordenates
            return: state of the cell 
        '''
        try:
            return self.cells[cell[1]][cell[0]].state
        except:
            return CellState.VISITED

    def get_cell_center(self,cell):
        return self.cells[cell[1]][cell[0]].get_cell_center()   
    
    def get_not_visited_cell(self):

        for row in range(self.rows):
            for col in range(self.cols):
                cell = self.cells[int(row)][int(col)]
                if cell.state > CellState.VISITED and  not cell.reserved: 
                    #print(f"{cell=}")
                    # set cell to reserved 
                    cell.reserved = True

                    center = cell.get_center() 
                    center[2] += OFFSET_ALTITUDE # default altitude 

                    return center

        return self.person.position

    def set_all_visited(self):
        size_block = 1
        z = self.z_offset
        for y in range(0, self.rows, size_block ):
            for x in range(0, self.cols, size_block ):
                cell = self.cells[int(x)][int(y)]
                if cell:
                    if cell.state != CellState.VISITED or cell.state != CellState.FOUND:
                        self.cells[int(x)][int(y)].change_state()


class Cell(object):
    def __init__(self, pBullet, position, state) -> None:
        self.pybullet = pBullet
        self.size = 1
        self.position = position
        self.state = state
        self.center_in_global_coord = [self.position[0] + self.size/2, self.position[1] + self.size/2]
        self.has_person = False

        self.reserved = False # Reserved means that a drone is going there to search

        self.id_cell = None

    def get_color(self):
        if(self.state == CellState.HIGH):
            return RED
        if(self.state == CellState.MEDIUM):
            return YELLOW
         
        return GRENN

    def draw_cell(self, z_offset = 0.1):

        width = 2.0
        n_squares = 4
        off = (1/2)/n_squares
        #self.id_duck = self.pybullet.loadURDF("duck_vhacd.urdf",
        #           self.get_center() ,
        #           self.pybullet.getQuaternionFromEuler([0, 0, 0])
        #           )

        self.id_cell= self.pybullet.loadURDF("cube.urdf",
                   self.get_center(),
                   self.pybullet.getQuaternionFromEuler([0, 0, 0])
                   )

        translation = self.position 
        self.pybullet.changeVisualShape(self.id_cell,-1 , rgbaColor = deepcopy(self.get_color())+[1])
        color = self.get_color()

        if(DRAW_CELLS):
            for i in range(n_squares):
                
                p1 = addPoints([0 + off ,0 + off ,z_offset], translation )
                p2 = addPoints([1 - off ,0 + off ,z_offset], translation )
                self.pybullet.addUserDebugLine(p1, p2, lineColorRGB=color, lineWidth=width, lifeTime=0)

                p3 = addPoints([1 - off,1 - off,z_offset], translation )
                self.pybullet.addUserDebugLine(p2, p3, lineColorRGB=color, lineWidth=width, lifeTime=0)

                p4 = addPoints([0 + off ,1 - off,z_offset], translation )
                self.pybullet.addUserDebugLine(p3, p4, lineColorRGB=color,  lineWidth=width, lifeTime=0)

                self.pybullet.addUserDebugLine(p4, p1, lineColorRGB=color, lineWidth=width, lifeTime=0)
                off += off
    
    def change_color(self):
        if(self.state == CellState.HIGH):
            color =  RED
        elif(self.state == CellState.MEDIUM):
            color = YELLOW
        elif(self.state == CellState.VISITED):
            color = GRAY
        elif(self.state ==  CellState.FOUND):
            color = BLUEE
        else:
            color = GRENN

        if(self.id_cell):
            self.pybullet.changeVisualShape(self.id_cell, -1 , rgbaColor = color+[1])

    def get_center(self, z_offset= 0.5):
        return [ self.position[0]+self.size/2, self.position[1]+self.size/2,  z_offset  ]

    def change_state(self, state = CellState.VISITED):
        if self.state != CellState.FOUND :
            self.state = state 
            self.change_color()

    def get_state(self):
        return self.state

    def get_cell_center(self):
        return self.center_in_global_coord


class Person(object):
    def __init__(self, grid_map, pybullet) -> None:
        '''
            Duck object represents a missing person
        '''

        self.grid_map = grid_map
        self.position = self.grid_map.get_random_cell() 
        self.cell = self.grid_map.get_cell(self.position)
        self.position[2] -= 0.3
        self.pybullet = pybullet

        self.cell.change_state(CellState.HIGH)
        self.cell.has_person = True

    def move(self, new_position):
        self.position = new_position
        

    def get_position(self):
        return self.position

    def draw(self):
        #self.id = self.pybullet.loadURDF("duck_vhacd.urdf",
        #          self.get_position() ,
        #          self.pybullet.getQuaternionFromEuler([0, 0, 0])
        #        )

        self.id = self.pybullet.loadURDF("r2d2.urdf",
                  self.get_position() ,
                  self.pybullet.getQuaternionFromEuler([0, 0, 0]),
                  globalScaling = 0.4
                  )