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

             Sokoban_features feature_tree(initial_map_ptr);
             feature_tree.solve();

            //  if (false) { // EXTERNAL TREE TESTER, NOT FOR USE IN ACTUAL PROGRAMMING
            //      Sokoban_features::feature_node* tree_root2 = feature_tree.insert_child(nullptr); // Reports an error in the terminal
            //      Sokoban_features::feature_node* s1 = feature_tree.insert_child(tree_root);
            //      Sokoban_features::feature_node* s2 = feature_tree.insert_child(tree_root);
            //      Sokoban_features::feature_node* s3 = feature_tree.insert_child(tree_root);
            //      Sokoban_features::feature_node* s4 = feature_tree.insert_child(s1);
            //      Sokoban_features::feature_node* s5 = feature_tree.insert_child(s1);
            //      Sokoban_features::feature_node* s6 = feature_tree.insert_child(s1);
            //      Sokoban_features::feature_node* s7 = feature_tree.insert_child(s1);
            //      Sokoban_features::feature_node* s8 = feature_tree.insert_child(s7);
            //      Sokoban_features::feature_node* s9 = feature_tree.insert_child(s7);
            //      Sokoban_features::feature_node* s10 = feature_tree.insert_child(s9);
            //      Sokoban_features::feature_node* s11 = feature_tree.insert_child(s10);
            //      Sokoban_features::feature_node* s12 = feature_tree.insert_child(s11);
            //      cout << "Depth of tree_root is: " << tree_root->depth << endl;
            //      cout << "Depth of s1 is: " << s1->depth << endl;
            //      cout << "Depth of s1's parent is: " << s1->parent->depth << endl;
             //
            //      if (tree_root == feature_tree.get_root_ptr()) {
            //          cout << "Roots match and are at " << tree_root << " (main) and " << feature_tree.get_root_ptr() << " (class)" << endl;
            //      }
             //
            //      if (tree_root->children.size() > 0)
            //          for (size_t i = 0; i < tree_root->children.size(); i++)
            //              cout << "Depth of tree_root's " << i+1 <<  " st child is: " << tree_root->children.at(i)->depth << endl;
            //      if (s1->children.size() > 0)
            //          for (size_t i = 0; i < s1->children.size(); i++)
            //              cout << "Depth of s1's " << i+1 <<  " st child is: " << s1->children.at(i)->depth << endl;
            //  }

            // Map *initial_map_ptr = &initial_map;
            //
            // cout << initial_map_ptr << endl;
            //
            // // Put code here
            // //cout << "tesrt" << endl;
            // Map second_map(initial_map);
            // //cout << &second_map << endl;
            // //cout << "tesrt" << endl;
            // //second_map.print_map();
            // second_map.parent_map->print_map();
            // Map third_map(second_map);
            // third_map.parent_map->parent_map->print_map();
            // //cout << second_map.parent_map << endl;
        }
    } else cout << "Please provide a map file and try again!" << endl;
    return 0;
}
