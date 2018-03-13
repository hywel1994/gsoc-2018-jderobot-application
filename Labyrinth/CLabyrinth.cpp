//
// Created by hywel on 18-3-13.
//

#include "CLabyrinth.h"

Labyrinth::Labyrinth(string input_file,string output_file) {

    input_file_path = input_file;
    output_file_path = output_file;

    read_from_input_file();

    map = new int *[row];
    for(int i = 0; i < row; i++){
        map[i] = new int [column];
    }

    input = new string *[row];
    for(int i = 0; i < row; i++){
        input[i] = new string [column];
    }

    //str = "##.######..#....#..#.##.##.#....##.#####";

    std::cout<< "point_map"<<std::endl;
    for(int i = 0; i<row; i++){
        for(int j = 0; j<column; j++){
            input[i][j] = str[i*column+j];

            std::stringstream ss;
            ss<<"#";
            string s1 = ss.str();
            if (input[i][j].compare(s1) == 0){
                map[i][j] = 0;
            }
            else
            {
                map[i][j] = 1;
            }
            //std::cout << input[i][j];
            std::cout << map[i][j];
        }
        std::cout<< std::endl;
    }


}

Labyrinth::~Labyrinth() {

    delete [] pointmap;
}


void Labyrinth::read_from_input_file(){
    std::ifstream inputFile(input_file_path);
    if (!inputFile.is_open()){
        std::cout<< "Error opening file"<<std::endl;
        return;
    }
    string s;
    string str_tmp = "";
    int num = 0;
    int itmp = 0;
    while(getline(inputFile,s)) {
        //cout << "Read from file: " << s << endl;
        std::stringstream ss;
        ss<<"";
        string s1 = ss.str();
        if (s.compare(s1) == 0){
            break;
        }
        num = s.length();
        str_tmp.append(s);
        itmp += 1;
    }

    row = itmp;
    column = num;
    str = str_tmp;
    std::cout << "row: " << row << std::endl;
    std::cout << "column: " << column << std::endl;
    std::cout << "str " << str << std::endl;
}

void Labyrinth::write_into_output_file(){
    std::ofstream outputFile (output_file_path);
    if (outputFile.is_open()) {

        for(int i = 0; i<row; i++) {
            for (int j = 0; j < column; j++) {
                outputFile << output[i][j];
            }
            outputFile << std::endl;
        }

        outputFile.close();
    }

}


void Labyrinth::classification_map(){

    std::cout<< "classification_map"<<std::endl;
    for (int i=0; i<row; i++){
        for (int j=0; j<column; j++){
            if (map[i][j]==1){

                vector<point> point_tmp;
                dfs_classification(i,j,point_tmp);

                for(int i=0; i<point_tmp.size(); ++i)
                    std::cout << "("<< point_tmp[i].x << "," << point_tmp[i].y << ") ";
                std::cout << std::endl;
                point_classification.push_back(point_tmp);
            }
        }
    }
    int num = point_classification.size();
    pointmap = new point_map[num];

}

void Labyrinth::dfs_classification(int x,int y, vector<point> &v_tmp){
    if (map[x][y] != 1){
        return;
    }
    map[x][y] = 0;
    point add;
    add.x = x;
    add.y = y;
    v_tmp.push_back(add);
    if ( x < row-1){
        dfs_classification(x+1,y,v_tmp);
    }
    if (y < column - 1){
        dfs_classification(x,y+1,v_tmp);
    }
    if ( x > 0){
        dfs_classification(x-1,y,v_tmp);
    }
    if ( y > 0){
        dfs_classification(x,y-1,v_tmp);
    }
}


bool Labyrinth::is_adjacent(point point1, point point2){
    int difference = std::abs(point1.x-point2.x)+std::abs(point1.y-point2.y);
    if(difference == 1) {
        return true;
    }
    else {
        return false;
    }
}

void Labyrinth::build_point_map(int seq){
    vector<point> point_tmp(point_classification[seq]);
    pointmap[seq].point_num = point_tmp.size();

    pointmap[seq].line_num = 0;
    pointmap[seq].connection = new int *[pointmap[seq].point_num];
    for(int i = 0; i<pointmap[seq].point_num;i++){
        pointmap[seq].connection[i] = new int [pointmap[seq].point_num];
    }
    //std::cout<<"point_num = "<<pointmap[seq].point_num<<std::endl;

    //std::cout<<"line_map"<<std::endl;
    for (int i = 0; i<pointmap[seq].point_num; i++) {
        for (int j = 0; j<pointmap[seq].point_num; j++) {
            pointmap[seq].connection[i][j] = pointmap[seq].connection[j][i] = 0;
            if (is_adjacent(point_tmp[i],point_tmp[j])){
                pointmap[seq].connection[i][j] = pointmap[seq].connection[j][i] = 1;
                pointmap[seq].line_num += 1;
            }
            //std::cout << pointmap[seq].connection[i][j];
        }
        //std::cout <<std::endl;
    }
}

void Labyrinth::dfs(int seq, int x, vector<int> v_tmp,vector<int> &path_start) {//以x为起点深搜

    v_tmp.push_back(x);
    vector<int> path_point(v_tmp);

    for(int i=0; i <pointmap[seq].point_num; i++){
        vector<int>::iterator iter=find(v_tmp.begin(),v_tmp.end(),i);
        if(iter == v_tmp.end() && pointmap[seq].connection[x][i] == 1) //x和y有边相连并且i没有访问过
        {
            vector<int> path_tmp;
            dfs(seq,i,v_tmp,path_tmp);
            if (path_point.size() < path_tmp.size()){
                path_point.assign(path_tmp.begin(),path_tmp.end());
            }
        }
    }

    path_start.assign(path_point.begin(),path_point.end());
}

void Labyrinth::find_max_path(){

    int sequence = point_classification.size();
    for(int seq = 0; seq<sequence; seq++){

        build_point_map(seq);
        for(int start_i = 0; start_i<pointmap[seq].point_num; start_i++){
            vector<int> v_tmp;
            vector<int> path_start;
            dfs(seq,start_i,v_tmp,path_start); //start search from strat_i

            if (pointmap[seq].path_max.size() < path_start.size()){
                pointmap[seq].path_max.assign(path_start.begin(),path_start.end());
            }
        }

//        std::cout<<"path_seq"<<std::endl;
//        for(int i=0; i<pointmap[seq].path_max.size(); ++i)
//            std::cout << pointmap[seq].path_max[i] << ' ';
//        std::cout << std::endl;


        vector<point> long_path;
        for (int i = 0; i<pointmap[seq].path_max.size(); ++i){
            int point_seq = pointmap[seq].path_max[i];
            vector<point> point_tmp(point_classification[seq]);
            point add;
            add.x = point_tmp[point_seq].x;
            add.y = point_tmp[point_seq].y;
            long_path.push_back(add);
            //std::cout << "("<< long_path[i].x << "," << long_path[i].y << ") ";
        }
        //std::cout << std::endl;

        if (longest_path.size() < long_path.size()){
            longest_path.assign(long_path.begin(),long_path.end());
        }
    }


    std::cout<< "longest pathway" <<std::endl;
    for(int i=0; i<longest_path.size(); ++i)
        std::cout << "("<< longest_path[i].x << "," << longest_path[i].y << ") ";
    std::cout << std::endl;

}

void Labyrinth::outfile() {
    output = input;
    for(int i=0; i<longest_path.size(); ++i)
    {
        std::stringstream ss;
        ss<<i;
        string s1 = ss.str();
        output[longest_path[i].x][longest_path[i].y] = s1;
    }

    std::cout<<"output map"<<std::endl;
    for(int i = 0; i<row; i++) {
        for (int j = 0; j < column; j++) {
            std::cout << output[i][j];
        }
        std::cout<<std::endl;
    }

    write_into_output_file();
}