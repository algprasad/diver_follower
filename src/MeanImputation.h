//
// Created by alg on 24/02/21.
//

#ifndef FILL_IN_MAT_MEANIMPUTATION_H
#define FILL_IN_MAT_MEANIMPUTATION_H

#include <vector>
#include <cmath>
#include <iostream>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

const double e = 2.71828;

struct CompletedVal{
    int i, j ;
    double val;
    CompletedVal(int i_, int j_, double val_): i(i_), j(j_), val(val_){}
};



void printMat(std::string title, std::vector<std::vector<double>>& mat ){
    std::cout<<title<<"  \n";
    for(int i =0; i< mat.size(); i++){
        for(int j =0; j<mat[i].size(); j++)
            std::cout<<mat[i][j]<< " ";
        std::cout<<std::endl;
    }

}

void printMat(std::string title, std::vector<double>& vec){
    std::cout<<title<<"  \n";
    for(auto el: vec) std::cout<<el<<" ";
    std::cout<<"\n";

}

void printMat(std::string title, std::vector<CompletedVal> vec){
    std::cout<<title<<"  \n";
    for(auto el: vec) std::cout<<el.val<< "  ";
    std::cout<<" \n";

}
void printMat(std::string title, std::vector<std::pair<int, int> > mat){
    std::cout<<title<<"  \n";
    for(auto el: mat) {
        std::cout<<el.first<<","<<el.second<<" \n";

    }
    std::cout<<"\n";

}
void identifyNans(std::vector<std::vector<double> >& mat, std::vector<CompletedVal>& completed_vals_vec, std::vector<std::pair<int, int> >& nan_indicies){
    for(int i =0; i< mat.size(); i++){
        for(int j =0; j < mat[i].size(); j++){
            if(std::isnan(mat[i][j])){
                nan_indicies.push_back(std::pair<int, int>(i,j));
            }
            else{
                completed_vals_vec.push_back(CompletedVal(i, j, mat[i][j]));
            }
        }
    }

    //printMat("Completed Vals", completed_vals_vec);
    //printMat("Nan indices", nan_indicies);

}
void identifyNans(grid_map::Matrix& mat, std::vector<CompletedVal>& completed_vals_vec, std::vector<std::pair<int, int> >& nan_indicies){
    for(int i =0; i< mat.rows(); i++){
        for(int j =0; j < mat.cols(); j++){
            if(std::isnan(mat(i,j))){
                nan_indicies.push_back(std::pair<int, int>(i,j));
            }
            else{
                completed_vals_vec.push_back(CompletedVal(i, j, mat(i,j)));
            }
        }
    }

    //printMat("Completed Vals", completed_vals_vec);
    //printMat("Nan indices", nan_indicies);

}

double getCompletedValue(int i, int j, std::vector<CompletedVal>& completed_vals_vec){
    double sum_weights = 0, sum_weighted_vals = 0;
    for(auto el: completed_vals_vec){
        double exponent = -1*std::max(abs(el.i - i), abs(el.j - j));
        double weight = pow(5, exponent);
        sum_weights+= weight;
        sum_weighted_vals+= weight*el.val;
    }
    double ans = sum_weighted_vals/sum_weights;
    return ans;

}

//TODO might need a converter or an overloaded function to use the matrix data structire
void fillIn(std::vector<std::vector<double> >& mat){
    std::vector<CompletedVal> completed_vals_vec;
    std::vector<std::pair<int, int> > nan_indices;
    //identify the nan values and the measured values
   identifyNans(mat, completed_vals_vec, nan_indices);

   //for each unidentified value
   for(auto el: nan_indices){
       mat[el.first][el.second] = getCompletedValue(el.first, el.second, completed_vals_vec);
   }
   printMat("Filled matrix" , mat);

}
void fillIn(grid_map::Matrix& mat){
    std::vector<CompletedVal> completed_vals_vec;
    std::vector<std::pair<int, int> > nan_indices;
    //identify the nan values and the measured values
    identifyNans(mat, completed_vals_vec, nan_indices);

    //for each unidentified value
    for(auto el: nan_indices){
        mat(el.first,el.second) = getCompletedValue(el.first, el.second, completed_vals_vec);
    }
    //printMat("Filled matrix" , mat);

}

void convert2Mat(std::vector<std::vector<double> >& mat, grid_map::Matrix& grid_mat){
    //for(int i =0; i < grid_mat.getSi)



}

void convert2GridMat(std::vector<std::vector<double> >& mat, grid_map::Matrix& grid_mat){
    for(int i =0; i< mat.size(); i++){
        for(int j =0; j< mat[i].size(); j++){
            mat[i][j] = grid_mat(i,j);
        }
    }

}



#endif //FILL_IN_MAT_MEANIMPUTATION_H

