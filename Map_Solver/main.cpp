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

void make_robot_commands(Sokoban_features::feature_node* solution_ptr, Sokoban_features &tree) {
    Sokoban_features::feature_node* current_ptr;
    current_ptr = solution_ptr;

    Sokoban_features::feature_node* parent_ptr;
    parent_ptr = solution_ptr->parent;

    Sokoban_features::feature_node* grandparent_ptr;
    grandparent_ptr = parent_ptr->parent;

    cout << tree.point_type(current_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y, worker);

    cout << "Robot commands: ";
    //LAST MOVE MUST BE A FORWARD SO START WITH THAT!
    cout << "F";
    while (grandparent_ptr != nullptr) {
        //cout << "Current: " << current_ptr->worker_pos.x << "," << current_ptr->worker_pos.y << " and dir " << current_ptr->worker_dir << endl;
        //cout << "Parent: " << parent_ptr->worker_pos.x << "," << parent_ptr->worker_pos.y << " and dir " << parent_ptr->worker_dir << endl;
        if (current_ptr->worker_pos.x == parent_ptr->worker_pos.x and current_ptr->worker_pos.y == parent_ptr->worker_pos.y) {
            // Turn
            if ( (current_ptr->worker_dir == NORTH and current_ptr->worker_dir == EAST)
                or (current_ptr->worker_dir == EAST and current_ptr->worker_dir == SOUTH)
                or (current_ptr->worker_dir == SOUTH and current_ptr->worker_dir == WEST)
                or (current_ptr->worker_dir == WEST and current_ptr->worker_dir == NORTH) ) {
                // CCW move
                cout << "L";
            } else {
                // CW move
                cout << "R";
            }
        } else {
            if ( (current_ptr->worker_dir == NORTH and current_ptr->worker_pos.x == parent_ptr->worker_pos.x and current_ptr->worker_pos.y == parent_ptr->worker_pos.y - 1 )
                or (current_ptr->worker_dir == EAST and current_ptr->worker_pos.x == parent_ptr->worker_pos.x - 1 and current_ptr->worker_pos.y == parent_ptr->worker_pos.y)
                or (current_ptr->worker_dir == SOUTH and current_ptr->worker_pos.x == parent_ptr->worker_pos.x and current_ptr->worker_pos.y == parent_ptr->worker_pos.y + 1)
                or (current_ptr->worker_dir == EAST and current_ptr->worker_pos.x == parent_ptr->worker_pos.x + 1 and current_ptr->worker_pos.y == parent_ptr->worker_pos.y) ) {
                // Forward move
                if ( (current_ptr->worker_dir == NORTH
                        and tree.point_type(current_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y-1, worker) == box
                        and tree.point_type(parent_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y, worker) == box
                        and (tree.point_type(grandparent_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y+1, worker) == freespace
                            or parent_ptr->worker_dir != grandparent_ptr->worker_dir))
                    or (current_ptr->worker_dir == EAST
                            and tree.point_type(current_ptr, current_ptr->worker_pos.x+1, current_ptr->worker_pos.y, worker) == box
                            and tree.point_type(parent_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y, worker) == box
                            and (tree.point_type(grandparent_ptr, current_ptr->worker_pos.x-1, current_ptr->worker_pos.y, worker) == freespace
                                or parent_ptr->worker_dir != grandparent_ptr->worker_dir))
                    or (current_ptr->worker_dir == SOUTH
                            and tree.point_type(current_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y+1, worker) == box
                            and tree.point_type(parent_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y, worker) == box
                            and (tree.point_type(grandparent_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y-1, worker) == freespace
                                or parent_ptr->worker_dir != grandparent_ptr->worker_dir))
                    or (current_ptr->worker_dir == WEST
                            and tree.point_type(current_ptr, current_ptr->worker_pos.x-1, current_ptr->worker_pos.y, worker) == box
                            and tree.point_type(parent_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y, worker) == box
                            and (tree.point_type(grandparent_ptr, current_ptr->worker_pos.x+1, current_ptr->worker_pos.y, worker) == freespace
                                or parent_ptr->worker_dir != grandparent_ptr->worker_dir)) ) {
                    cout << "A";
                } else
                    cout << "F";
                // DEPLOY is missing
            } else {
                cout << "B";
            }
        }
        current_ptr = parent_ptr;
        parent_ptr = grandparent_ptr;
        grandparent_ptr = grandparent_ptr->parent;
    }

    cout << endl;
}

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
                     make_robot_commands(feature_tree.get_goal_node_ptr(), feature_tree);
                 } else {
                     feature_tree.print_info("No solution was found using Breadth-first searching");
                     feature_tree.print_info("Visited "+to_string(feature_tree.get_closed_list_size())+" nodes");
                 }

             }
        }
    } else cout << "Please provide a map file and try again!" << endl;
    return 0;
}
