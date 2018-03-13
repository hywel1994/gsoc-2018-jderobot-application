//
// Created by hywel on 18-3-10.
//

#include "CLabyrinth.h"
#include <iostream>
#include <string>
using std::string;

int main() {

    char quickly_start;
    string input_file;
    string output_file;

    std::cout << "quickly start (y/n) :";
    std::cin >> quickly_start;
    if(quickly_start == 'y'){
        input_file = "../input.txt";
        output_file = "../output.txt";
    }
    else{
        char in_file[50];
        char out_file[50];
        std::cout << "input file path:";
        std::cin >> in_file;
        std::cout << "output file path:" ;
        std::cin >> out_file;
        input_file = in_file;
        output_file = out_file;
    }

    Labyrinth lab(input_file,output_file);
    lab.classification_map();
    lab.find_max_path();
    lab.outfile();
}
