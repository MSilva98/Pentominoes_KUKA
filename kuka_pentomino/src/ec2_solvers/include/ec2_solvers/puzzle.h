
#ifndef EC2_SOLVERS_PUZZLE_H_
#define EC2_SOLVERS_PUZZLE_H_

#include <vector>
#include <tuple>
#include <string>

namespace ec2{
    class puzzle
    {
    public:
        bool charInVector(std::vector<char> v, char c);
        std::vector<char> getPiecesUsed(std::vector<std::vector<char> > game_sol);
        std::vector<std::vector<char>> readMatrixFromFile(std::string name_file);
        std::vector<std::vector<char>> rotate90clockwise(std::vector<std::vector<char>> temp);
        std::vector<std::vector<char>> getSubArray(std::vector<std::vector<char>> arr, int idxRow, int idxCol, int rows, int cols);
        bool matchArrayPiece(std::vector<std::vector<char>> arrGame, std::vector<std::vector<char>> arrPiece, char typePiece);
        void print_array(std::vector<std::vector<char>> v);
        std::vector<std::tuple <char, int, int, double>> getSolution(std::string filename);
        std::tuple<int,int> getPointGrasp(std::vector<std::vector<char>> arrPiece, char typePiece);
        std::vector<std::tuple <char, int, int, double>> orderByDistance(std::vector<std::tuple <char, int, int, double>> vec);
    };
}

#endif
