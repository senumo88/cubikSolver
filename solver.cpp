#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <algorithm>
using namespace std;

/*
int main()
{
    string solucion[] = {
        "UF", "UR", "UB", "UL",
        "DF", "DR", "DB", "DL",
        "FR", "FL",
        "BR", "BL",
        "UFR", "URB", "UBL", "ULF",
        "DRF", "DFL", "DLB", "DBR"
    };
    ///FRONT 1  F1 F2
    string start_FRONT1[] = {
        "LF", "UR", "UB", "UL",
        "RF", "DR", "DB", "DL",
        "FU", "FD",
        "BR", "BL",
        "LFU", "URB", "UBL", "LDF",
        "RUF", "RFD", "DLB", "DBR"
    };
    ///FRONT 2  F2
    string start_FRONT2[] = {
        "DF", "UR", "UB", "UL",
        "UF", "DR", "DB", "DL",
        "FL", "FR",
        "BR", "BL",
        "DFL", "URB", "UBL", "DRF",
        "ULF", "UFR", "DLB", "DBR"
    };
    ///UP 1 U1, U2
    string start_UP[] = {
        "UR", "UB", "UL", "UF",
        "DF", "DR", "DB", "DL",
        "FR", "FL",
        "BR", "BL",
        "URB", "UBL", "ULF", "UFR",
        "DRF", "DFL", "DLB", "DBR"
    };


    vector<string> moves(solve(start_FRONT2, solucion));
    cout <<"MOVIMIENTOS: ";
    for (auto i : moves) {
        cout << i << " ";
    }
    cout << endl;

    return 0;
}
*/
