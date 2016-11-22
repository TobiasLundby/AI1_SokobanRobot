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

#include "common.cpp"
#include "Map.hpp"
#include "Sokoban_features.hpp"

using namespace std;

int main(int argc,  char **argv) {
    if (argc >= 2) { // Accept only one file
        string map_file_name = argv[1]; //filename
        Map initial_map;
        Map* initial_map_ptr = &initial_map;
        if (initial_map.load_map_from_file(map_file_name)) {
             initial_map.print_map();
             initial_map.print_info();
             initial_map.print_goals();
             initial_map.print_boxes();
             cout << endl;
             if (initial_map.create_deadlock_free_map()) {
                 initial_map.print_map_simple(worker);
                 initial_map.print_map_simple(box);
                 Sokoban_features feature_tree(initial_map_ptr);
                 if (feature_tree.solve(BF)) {  // Solve using Breadth-first searching
                     feature_tree.print_info("Solved using Breadth-first searching");
                     feature_tree.print_info("Nodes visited "+to_string(feature_tree.get_closed_list_size()));
                     feature_tree.print_info("Nodes not visited "+to_string(feature_tree.get_open_list_size()));
                     feature_tree.print_branch_up(feature_tree.get_goal_node_ptr());
                     //make_robot_commands(feature_tree.get_goal_node_ptr(), feature_tree);
                 } else {
                     feature_tree.print_info("No solution was found using Breadth-first searching");
                     feature_tree.print_info("Visited "+to_string(feature_tree.get_closed_list_size())+" nodes");
                 }

             }
        }
    } else cout << "Please provide a map file and try again!" << endl;
    return 0;
}
