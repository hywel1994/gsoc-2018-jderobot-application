//
// Created by hywel on 18-3-13.
//

#ifndef MY_WS_CLABYRINTH_H
#define MY_WS_CLABYRINTH_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <sstream>
#include <fstream>

using std::string;
using std::vector;


struct point_map{
    int point_num, line_num;
    int **connection;
    vector<int> path_max;
};

struct point{
    int x;
    int y;
};

class Labyrinth {
public:

    Labyrinth(string input_file,string output_file);
    ~Labyrinth();

    void read_from_input_file();

    void write_into_output_file();

    //classify the point in the map
    void classification_map();

    //using dfs method classify the point
    void dfs_classification(int x,int y, vector<point> &v_tmp);

    //judge two point is adjacent
    bool is_adjacent(point point1, point point2);

    // set a two-digit array to storage the line map between the points
    void build_point_map(int seq);

    //using dfs method to calculate the longest pathway with one start point and one point group
    void dfs(int seq, int x, vector<int> v_tmp,vector<int> &path_start);

    //find the longest pathway
    void find_max_path();

    //output the result
    void outfile();

private:

    string input_file_path;
    string output_file_path;

    point_map *pointmap;

    int row;
    int column;
    int **map;

    string str;
    string **input;
    string **output;

    vector<vector<point> > point_classification;

    vector<point> longest_path;

};



#endif //MY_WS_CLABYRINTH_H
