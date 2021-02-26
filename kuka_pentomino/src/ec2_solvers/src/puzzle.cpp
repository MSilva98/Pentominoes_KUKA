#include <ec2_solvers/puzzle.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <tuple>       

using namespace std;
namespace ec2{

    vector<tuple <char, int, int, double>> puzzle::getSolution(){

        vector<char> names{'F', 'V', 'N', 'P', 'U', 'X'};
        vector<vector<char> > game_sol  = readMatrixFromFile("sol2-pentamino.txt");
        vector<vector<vector<char> >> pieces_temps; 
        pieces_temps.push_back(readMatrixFromFile("templates/f_matrix_template.txt"));
        pieces_temps.push_back(readMatrixFromFile("templates/v_matrix_template.txt"));
        pieces_temps.push_back(readMatrixFromFile("templates/n_matrix_template.txt"));
        pieces_temps.push_back(readMatrixFromFile("templates/p_matrix_template.txt"));
        pieces_temps.push_back(readMatrixFromFile("templates/u_matrix_template.txt"));
        pieces_temps.push_back(readMatrixFromFile("templates/x_matrix_template.txt"));

        vector<tuple <char, int, int, double>> sol_positions; // pieceType, idxRows, idxCols, angle

        //iterate pieces
        for (int pieceIdx = 0; pieceIdx < pieces_temps.size(); ++pieceIdx)
        {
            vector<vector<char>> piece_arr = pieces_temps[pieceIdx];
            //cout << "TRY PIECE " << names[pieceIdx] << endl;
            bool status = true;
            int count = 0;
            double angle = 0;
            while(status){
                for (int i = 0; i < game_sol.size() - piece_arr.size() + 1; ++i)
                {
                    for (int j = 0; j < game_sol[0].size() - piece_arr[0].size() + 1; ++j)
                    {
                        vector<vector<char>> game_sub_arr = getSubArray(game_sol,i,j,piece_arr.size(), piece_arr[0].size());
                        if(matchArrayPiece(game_sub_arr,piece_arr,names[pieceIdx])){
                            //cout << "PIECE " << names[pieceIdx] << " RECOGNIZED AT " << i << ", " << j << endl;
                            tuple<int, int> pos = getPointGrasp(piece_arr, names[pieceIdx] );
                            sol_positions.push_back(make_tuple(names[pieceIdx], get<0>(pos) + i, get<1>(pos) + j, angle));
                            status = false;
                            break;
                        }
                    }
                    if(!status){
                        break;
                    }
                }
                //if dont match rotate piece and repeat
                if(status){
                    piece_arr = rotate90clockwise(piece_arr);
                    angle = angle + 90;
                    count++;
                }
                if(count > 3){
                    break;
                }
            }
        }

        // vector<double> angleToGrabTemp{0, 90, 90, 90, 90, 0};

        // for (size_t i = 0; i < sol_positions.size(); i++)
        // {
        //     for (size_t j = 0; i < names.size(); i++)
        //     {
        //         if(get<0>(sol_positions[i]) == names[j]){
        //             double new_ang = get<3>(sol_positions[i]) - angleToGrabTemp[j];
        //             sol_positions[i] = tuple <char, int, int, double>(get<0>(sol_positions[i]), get<1>(sol_positions[i]), get<2>(sol_positions[i]),angle );
        //         }
        //     }
        // }
        

        //order
        sol_positions = orderByDistance(sol_positions);

        return sol_positions;
    }

    vector<tuple <char, int, int, double>> puzzle::orderByDistance(vector<tuple <char, int, int, double>> vec){
        tuple <char, int, int, double> temp;
        for(int i = 0; i<vec.size(); i++) {
        for(int j = i+1; j<vec.size(); j++)
        {
            if(get<1>(vec[j]) + get<2>(vec[j])  < get<1>(vec[i]) + get<2>(vec[i])) {
                temp = vec[i];
                vec[i] = vec[j];
                vec[j] = temp;
            }
        }
        }
        return  vec;
    }

    tuple<int,int> puzzle::getPointGrasp(vector<vector<char>> arrPiece, char typePiece){
        for (int i = 0; i < arrPiece.size(); ++i)
        {
            for (int j = 0; j < arrPiece[0].size(); ++j)
            {
                if(arrPiece[i][j] == typePiece ){
                    return make_tuple(i,j);
                }
            }
        }
    }

    void puzzle::print_array(vector<vector<char>> v){
        for (int i = 0; i < v.size(); ++i)
        {
            for (int j = 0; j < v[i].size(); ++j)
            {
                cout << v[i][j];
            }
            cout << endl;
        }
    }

    bool puzzle::matchArrayPiece(vector<vector<char>> arrGame, vector<vector<char>> arrPiece, char typePiece){
        bool result = true;
        if(typePiece == 'F'){
            for (int i = 0; i < arrPiece.size(); ++i)
            {
                for (int j = 0; j < arrPiece[0].size(); ++j)
                {
                    if(arrPiece[i][j] == 'f' || arrPiece[i][j] == 'F' ){
                        if(arrGame[i][j] != 'F' ){
                            return false;
                        }
                    }
                }
            }
        }else if(typePiece == 'V'){
            for (int i = 0; i < arrPiece.size(); ++i)
            {
                for (int j = 0; j < arrPiece[0].size(); ++j)
                {
                    if(arrPiece[i][j] == 'v' || arrPiece[i][j] == 'V' ){
                        if(arrGame[i][j] != 'V' ){
                            return false;
                        }
                    }
                }
            }
        }else if(typePiece == 'N'){
            for (int i = 0; i < arrPiece.size(); ++i)
            {
                for (int j = 0; j < arrPiece[0].size(); ++j)
                {
                    if(arrPiece[i][j] == 'n' || arrPiece[i][j] == 'N' ){
                        if(arrGame[i][j] != 'N' ){
                            return false;
                        }
                    }
                }
            }
        }else if(typePiece == 'P'){
            for (int i = 0; i < arrPiece.size(); ++i)
            {
                for (int j = 0; j < arrPiece[0].size(); ++j)
                {
                    if(arrPiece[i][j] == 'p' || arrPiece[i][j] == 'P' ){
                        if(arrGame[i][j] != 'P' ){
                            return false;
                        }
                    }
                }
            }
        }else if(typePiece == 'U'){
            for (int i = 0; i < arrPiece.size(); ++i)
            {
                for (int j = 0; j < arrPiece[0].size(); ++j)
                {
                    if(arrPiece[i][j] == 'u' || arrPiece[i][j] == 'U' ){
                        if(arrGame[i][j] != 'U' ){
                            return false;
                        }
                    }
                }
            }
        }else if(typePiece == 'X'){
            for (int i = 0; i < arrPiece.size(); ++i)
            {
                for (int j = 0; j < arrPiece[0].size(); ++j)
                {
                    if(arrPiece[i][j] == 'x' || arrPiece[i][j] == 'X' ){
                        if(arrGame[i][j] != 'X' ){
                            return false;
                        }
                    }
                }
            }
        }
        return true;
    }

    vector<vector<char>> puzzle::getSubArray(vector<vector<char>> arr, int idxRow, int idxCol, int rows, int cols)
    {
        vector<vector<char>> result(rows,vector<char>(cols));
        int k = 0;
        int l = 0;
        for (int i=idxRow; i <idxRow +rows ;  i++)
        {
            for (int j=idxCol; j< idxCol + cols; j++)
            {
                result[k][l] = arr[i][j];
                l++;
            }
            k++;
            l = 0;
        }
        return result;
    }

    vector<vector<char>> puzzle::rotate90clockwise(vector<vector<char>> temp){
        int rows = temp[0].size();
        int cols = temp.size();

        vector<vector<char>> result(rows,vector<char>(cols));

        for (int i = 0; i < temp.size(); ++i)
        {
            for (int j = 0; j < temp[i].size(); ++j)
            {
                result[j][i] = temp[temp.size() -1 -i][j];
            }
        }
        return result;
    }


    vector<vector<char>> puzzle::readMatrixFromFile(string name_file){
        vector<vector<char> > v;
        std::ifstream file(name_file);
        if (file.is_open()) {
            std::string line;
            while (std::getline(file, line)) {
                vector<char> temp;
                for (int i = 0; i < line.size(); i++) {
                    temp.push_back(line[i]);
                }
                v.push_back(temp);
            }
            file.close();
        }
        return v;
    } 
}
