//
//  main.hpp
//  AI1_Sokoban-solver_MM-TL
//
//  Created by Tobias Lundby on 19/09/16.
//  All files are licenced under the BSD 3-Clause (see LICENSE.md)
//

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

using namespace std;

#include "Map.hpp"
#include "Sokoban_features.hpp"

int main(int argc,  char **argv) {
    if (argc >= 2) { // Accept only one file
        string map_file_name = argv[1]; //filename
        Map initial_map;
        if (initial_map.load_map_from_file(map_file_name)) {
            //initial_map.print_map();
            //initial_map.print_info();

            Map *initial_map_ptr = &initial_map;

            cout << initial_map_ptr << endl;

            // Put code here
            //cout << "tesrt" << endl;
            Map second_map(initial_map);
            //cout << &second_map << endl;
            //cout << "tesrt" << endl;
            //second_map.print_map();
            second_map.parent_map->print_map();
            Map third_map(second_map);
            third_map.parent_map->parent_map->print_map();
            //cout << second_map.parent_map << endl;
        }
    } else cout << "Please provide a map file and try again!" << endl;
    return 0;
}
