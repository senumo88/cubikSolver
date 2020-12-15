#include <vector>
#include <string>
#include "cube.h"
#include "prune.h"
#include "edges.h"
#include "corners.h"

class OptimalSolver
{
private:
    long long nodeCnt;
    std::string solution;
    std::string moveNames[6] = {"U", "D", "R", "L", "F", "B"};
    std::string moveCntNames[3] = {" ", "2", "'"};
    Prune<Corners> cPruning;
    Prune<Edges> ePruning;
    int edgePruningSize;

public:
    OptimalSolver(int edgePruningSize);
    char solve(std::string scramble);
    char IDA(Cube &cube);
    
private: 
    bool treeSearch(Cube &cube, int depth, int lastMove);
};
