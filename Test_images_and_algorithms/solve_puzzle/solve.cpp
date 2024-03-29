#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <tuple>       

using namespace std;

vector<vector<char>> readMatrixFromFile(string name_file, int rows, int cols );
vector<vector<char>> rotate90clockwise(vector<vector<char>> temp);
vector<vector<char>> getSubArray(vector<vector<char>> arr, int idxRow, int idxCol, int rows, int cols);
bool matchArrayPiece(vector<vector<char>> arrGame, vector<vector<char>> arrPiece, char typePiece);
void print_array(vector<vector<char>> v);
vector<tuple <char, int, int, double>> solve();
tuple<int,int> getPointGrasp(vector<vector<char>> arrPiece, char typePiece);

int main() {
    vector<tuple <char, int, int, double>> solution = solve();

    for (int i = 0; i < solution.size(); ++i)
    {
        cout << "PIECE " << get<0>(solution[i])<< " RECOGNIZED AT " << get<1>(solution[i]) << ", " << get<2>(solution[i]) << " angle: " << get<3>(solution[i])  << endl;
    }

    // x = innerCorner.x + sideSquare* get<1>(solution[i]) - sideSquare/2 + offset_de_segurança*get<1>(solution[i]) ;
    // y = innerCorner.y + sideSquare* get<2>(solution[i]) - sideSquare/2 + offset_de_segurança*get<2>(solution[i]) ;

}

vector<tuple <char, int, int, double>> solve(){

    vector<char> names{'F', 'V', 'N', 'P', 'U', 'X'};
    vector<vector<char> > game_sol  = readMatrixFromFile("sol2-pentamino.txt", 5, 6);
    vector<vector<vector<char> >> pieces_temps; 
    pieces_temps.push_back(readMatrixFromFile("f_matrix_template.txt", 3, 3));
    pieces_temps.push_back(readMatrixFromFile("v_matrix_template.txt", 3, 3));
    pieces_temps.push_back(readMatrixFromFile("n_matrix_template.txt", 2, 4));
    pieces_temps.push_back(readMatrixFromFile("p_matrix_template.txt", 2, 3));
    pieces_temps.push_back(readMatrixFromFile("u_matrix_template.txt", 2, 3));
    pieces_temps.push_back(readMatrixFromFile("x_matrix_template.txt", 3, 3));

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
    return sol_positions;
}

tuple<int,int> getPointGrasp(vector<vector<char>> arrPiece, char typePiece){
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

void print_array(vector<vector<char>> v){
    for (int i = 0; i < v.size(); ++i)
    {
        for (int j = 0; j < v[i].size(); ++j)
        {
            cout << v[i][j];
        }
        cout << endl;
    }
}

bool matchArrayPiece(vector<vector<char>> arrGame, vector<vector<char>> arrPiece, char typePiece){
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

vector<vector<char>> getSubArray(vector<vector<char>> arr, int idxRow, int idxCol, int rows, int cols)
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

vector<vector<char>> rotate90clockwise(vector<vector<char>> temp){
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


vector<vector<char>> readMatrixFromFile(string name_file, int rows, int cols ){
    vector<vector<char> > v(rows,vector<char>(cols));

    ifstream fp(name_file);
    if (! fp) {
        cout << "Error, file couldn't be opened" << endl; 
    }    
    for(int row = 0; row <  v.size(); row++) {  // stop loops if nothing to read
       for(int column = 0; column < v[row].size(); column++){
            fp >> v[row][column];
            if ( ! fp ) {
               cout << "Error reading file for element " << row << "," << column << endl; 
            }
        }
    }
    fp.close();
    return v;
} 
