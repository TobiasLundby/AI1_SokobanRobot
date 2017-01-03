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
#include <iomanip>

#include "common.cpp"
#include "Map.hpp"
#include "Sokoban_features.hpp"

using namespace std;

int determine_robot_move(Sokoban_features::feature_node* current_ptr, Sokoban_features::feature_node* parent_ptr, Sokoban_features &tree) {
    if (current_ptr->worker_dir == parent_ptr->worker_dir) {
        if (current_ptr->worker_dir == NORTH) {
            if (tree.point_type(parent_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y+1, worker) == worker) {
                // Forwards move
                return F;
            } else {
                // Backwards move
                return B;
            }
        } else if (current_ptr->worker_dir == EAST) {
            if (tree.point_type(parent_ptr, current_ptr->worker_pos.x-1, current_ptr->worker_pos.y, worker) == worker) {
                // Forwards move
                return F;
            } else {
                // Backwards move
                return B;
            }
        } else if (current_ptr->worker_dir == SOUTH) {
            if (tree.point_type(parent_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y-1, worker) == worker) {
                // Forwards move
                return F;
            } else {
                // Backwards move
                return B;
            }
        } else if (current_ptr->worker_dir == WEST) {
            if (tree.point_type(parent_ptr, current_ptr->worker_pos.x+1, current_ptr->worker_pos.y, worker) == worker) {
                // Forwards move
                return F;
            } else {
                // Backwards move
                return B;
            }
        }
    } else {
        // A turn is detected
        if (   (current_ptr->worker_dir == NORTH and parent_ptr->worker_dir == EAST)
            or (current_ptr->worker_dir == EAST and parent_ptr->worker_dir == SOUTH)
            or (current_ptr->worker_dir == SOUTH and parent_ptr->worker_dir == WEST)
            or (current_ptr->worker_dir == WEST and parent_ptr->worker_dir == NORTH)    )
        {
            return L; // CCW
        }
        else
        {
            return R; // CW
        }
    }
    return 0; // Not a valid move!
}

bool box_inFrontOf_robot (Sokoban_features::feature_node* current_ptr, Sokoban_features &tree) {
    if (current_ptr->worker_dir == NORTH) {
        if (tree.point_type(current_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y-1, worker)==box)
            return true;
        else
            return false;
    } else if (current_ptr->worker_dir == EAST) {
        if (tree.point_type(current_ptr, current_ptr->worker_pos.x+1, current_ptr->worker_pos.y, worker)==box)
            return true;
        else
            return false;
    } else if (current_ptr->worker_dir == SOUTH) {
        if (tree.point_type(current_ptr, current_ptr->worker_pos.x, current_ptr->worker_pos.y+1, worker)==box)
            return true;
        else
            return false;
    } else if (current_ptr->worker_dir == WEST) {
        if (tree.point_type(current_ptr, current_ptr->worker_pos.x-1, current_ptr->worker_pos.y, worker)==box)
            return true;
        else
            return false;
    }
    return false;
}

void make_robot_commands(Sokoban_features::feature_node* solution_ptr, Sokoban_features &tree) {
    vector< Sokoban_features::feature_node* > branch;
    while (solution_ptr != nullptr) {
		branch.push_back(solution_ptr);
		solution_ptr = solution_ptr->parent;
	}

    Sokoban_features::feature_node* grandparent_ptr;
    Sokoban_features::feature_node* parent_ptr;
    Sokoban_features::feature_node* current_ptr;

    grandparent_ptr = branch.back();
    branch.pop_back();

    parent_ptr = branch.back();
    branch.pop_back();

    current_ptr = branch.back();
    branch.pop_back();

    int move = 0;
    bool worker_attached_to_box = false;

    string robot_commands;

    cout << "Robot commands: ";

    // Special case for first move below
    move = determine_robot_move(parent_ptr,grandparent_ptr,tree);
    if ( move==F and box_inFrontOf_robot(grandparent_ptr,tree) and box_inFrontOf_robot(parent_ptr,tree) ) {
        cout << "A";
        robot_commands += "A";
        worker_attached_to_box = true;
    } else if (move==F) {
        cout << "F";
        robot_commands += "F";
    } else if (move==B) {
        cout << "B";
        robot_commands += "B";
    } else if (move==L) {
        cout << "L";
        robot_commands += "L";
    } else if (move==R) {
        cout << "R";
        robot_commands += "R";
    }
    // General conversion
    while (branch.size()) {
        grandparent_ptr = parent_ptr;
        parent_ptr = current_ptr;
        current_ptr = branch.back();
        branch.pop_back();
        move = determine_robot_move(parent_ptr,grandparent_ptr,tree);

        if (move==F) {
            if (!worker_attached_to_box) { // worker is not currently pushing, but check for it!
                if ( box_inFrontOf_robot(grandparent_ptr,tree) and box_inFrontOf_robot(parent_ptr,tree) ) {
                    cout << "A";
                    robot_commands += "A";
                    worker_attached_to_box = true;
                } else {
                    cout << "F";
                    robot_commands += "F";
                }
            } else {
                cout << "F";
                robot_commands += "F";
            }
        } else if (worker_attached_to_box) {
            cout << "D";
            robot_commands += "D";
            worker_attached_to_box = false;
        }
        if (move==B) {
            cout << "B";
            robot_commands += "B";
        } else if (move==L) {
            cout << "L";
            robot_commands += "L";
        } else if (move==R) {
            cout << "R";
            robot_commands += "R";
        }
    }
    // Special case for last move; it should be a forwards move and in the current case that becomes deploy
    move = determine_robot_move(current_ptr,parent_ptr,tree);
    if (move==F){
        if (!worker_attached_to_box) { // worker is not currently pushing, but check for it!
            if ( box_inFrontOf_robot(parent_ptr,tree) and box_inFrontOf_robot(current_ptr,tree) ) {
                cout << "AD";
                robot_commands += "AD";
                worker_attached_to_box = true;
            }
        } else {
            cout << "D";
            robot_commands += "D";
        }
    } else if (move==B) {
        cout << "B";
        robot_commands += "B";
    } else if (move==L) {
        cout << "L";
        robot_commands += "L";
    } else if (move==R) {
        cout << "R";
        robot_commands += "R";
    }
    // Print line break to ensure proper terminal layout
    cout << endl;

    ofstream myfile;
    myfile.open ("robot_string.txt");
    myfile << robot_commands;
    myfile.close();
}

int main(int argc,  char **argv) {
    if (argc >= 2) { // Accept only one file
        string map_file_name = argv[1]; //filename
        Map initial_map;
        Map* initial_map_ptr = &initial_map;
        if (initial_map.load_map_from_file(map_file_name)) {
             string user_input;
            //  cout << "[INPUT] Print map and info (y/n): ";
            //  getline(cin, user_input);
            //  if (user_input == "y") {
            //      initial_map.print_map();
            //      initial_map.print_info();
            //      initial_map.print_goals();
            //      initial_map.print_boxes();
            //      cout << endl;
            //  }
            bool found_solution;
            int solution_steps = 0;
             if (initial_map.create_deadlock_free_map()) {
                 initial_map.print_map_simple(worker);
                 initial_map.print_map_simple(box);
                 Sokoban_features feature_tree(initial_map_ptr);
                 feature_tree.print_info("Starting search");
                 long long time_start = feature_tree.currentTimeUs();
                 long long time_end;
                 int solver_type = BF;
                 initial_map.create_wavefront_map(); // Generate wavefront maps
                 if (feature_tree.solve(solver_type, 10000000)) {  // Solve using Breadth-first searching
                     time_end = feature_tree.currentTimeUs();
                     cout << "Start time was " << time_start << " and end time was " << time_end << " and diff is "<< time_end-time_start << endl;
                     found_solution = true;
                     solution_steps = feature_tree.get_goal_node_ptr()->depth;
                     feature_tree.print_info("Solved");
                     feature_tree.print_info("Nodes visited "+to_string(feature_tree.get_closed_list_size()));
                     feature_tree.print_info("Nodes not visited "+to_string(feature_tree.get_open_list_size()));
                     //make_robot_commands(feature_tree.get_goal_node_ptr(), feature_tree);
                     cout << endl;

                    //  cout << "[INPUT] Print solution (y/n): ";
                    //  getline(cin, user_input);
                    //  if (user_input == "y") {
                    //      feature_tree.print_branch_up(feature_tree.get_goal_node_ptr());
                    //  }
                 } else {
                     time_end = feature_tree.currentTimeUs();
                     cout << "Start time was " << time_start << " and end time was " << time_end << " and diff is "<< time_end-time_start << endl;
                     found_solution = false;
                     feature_tree.print_info("No solution was found using the selected search algorithm");
                     feature_tree.print_info("Visited "+to_string(feature_tree.get_closed_list_size())+" nodes");
                 }
                 ofstream timing_data;
                 timing_data.open ("timing_data.csv",fstream::app|fstream::out);
                 //timing_data << "t_start,t_end,t_diff,closed_list,open_list,solver_type,solved,steps\n"; // Only used for saving the file the first time, otherwise it just appends
                 timing_data << setprecision(8) << time_start << "," << time_end << "," << time_end-time_start << setprecision(0) << "," << feature_tree.get_closed_list_size() << "," << feature_tree.get_open_list_size() << "," << solver_type << "," << found_solution << "," << solution_steps << "\n";
                 // remove set precision!!! no effect here!
             }
        }
    } else cout << "Please provide a map file and try again!" << endl;
    return 0;
}
