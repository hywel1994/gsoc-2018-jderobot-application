#!/usr/bin/env python

import numpy as np
import time
import random


class ConwayGOF():
    def __init__(self,row = 10,column = 10,sleep = 1,type = 0):
        self.row = row
        self.column = column
        self.sleep = sleep
        self.type = type
        self.time = 0
        self.map = np.zeros((self.row,self.column))

        self.type_input()


    def input(self,map):
        if map.shape[0] == self.row and map.shape[1] == self.column:
            self.map = map
        else:
            print ('error map row or column is not correct')
            return


    def type_input(self):
        if self.type == 0:
            map = np.random.randint(0,2, (self.row,self.column))
            self.input(map)
        elif self.type == 2:
            map = np.array([[0,0,0,0,0,0,0,0],
                            [0,0,0,1,1,0,0,0],
                            [0,0,0,0,1,0,0,0],
                            [0,0,0,1,0,0,0,0],
                            [0,0,0,1,0,0,0,0],
                            [0,0,0,1,1,0,0,0],
                            [0,0,0,0,1,0,0,0],
                            [0,0,0,0,0,0,0,0]])
            self.row = 8
            self.column = 8
            self.input(map)

    def start(self):

        while 1:
            self.calculate()


    def calculate(self):
        self.time += 1

        cal = np.zeros((self.row,self.column))
        mapBig = np.zeros((self.row+2,self.column+2))
        for row in range(self.row):
            for column in range(self.column):
                mapBig[row+1][column+1] = self.map[row][column]
        #calu
        for row in range(self.row):
            for column in range(self.column):
                cal[row][column] = mapBig[row][column] + mapBig[row+1][column] + mapBig[row+2][column] + mapBig[row][column+1] + mapBig[row+2][column+1] + mapBig[row][column+2] + mapBig[row+1][column+2] + mapBig[row+2][column+2]
                if cal[row][column] < 2 or cal[row][column] > 3:
                    self.map[row][column] = 0
                elif cal[row][column] == 3:
                    self.map[row][column] = 1
        self.draw_map()
        time.sleep(self.sleep)


    def draw_map(self):
        print ('the time of Conway Game of life : %d \n' %self.time)
        for row in range(self.row):
            for columns in range(self.column):
                if self.map[row][columns] == 0:
                    print('  .', end = '')
                elif self.map[row][columns] == 1:
                    print('  *', end = '')
                else:
                    print('error')
                    return
            print('\n')




