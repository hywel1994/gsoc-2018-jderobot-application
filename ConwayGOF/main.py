#!/usr/bin/env python


from ConwayGoF import *
import json
import numpy as np

if __name__ == '__main__':

    print ('------- Conway Game of life -------')

    quick_start = input("quick_start (y/n):")

    if quick_start == 'y':
        gof = ConwayGOF()
    else:
        type = int(input("type (random : 0, json : 1, other : 2):"))
        if type == 0:
            row = int(input("row :"))
            column = int(input("column :"))
            sleep = float(input("sleep time :"))

            gof = ConwayGOF(row = row,column = column,sleep = sleep,type = type)
        elif type == 1:
            fjson = open("config.json", encoding = 'utf-8')
            setting = json.load(fjson)
            row = setting['row']
            column = setting['column']
            sleep = setting['sleep']
            map = setting['map']
            map_new = np.zeros((row,column))
            for r in range(row):
                for c in range(column):
                    map_new[r][c] = int(map[r][c])

            #print (map[0][0])
            gof = ConwayGOF(row = row,column = column,sleep = sleep,type = type)
            gof.input(map_new)

        elif type == 2:
            gof = ConwayGOF(type = 2)

        else:
            print ('error')

    gof.start()
