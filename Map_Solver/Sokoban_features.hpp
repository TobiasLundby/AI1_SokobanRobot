//
//  MSokoban_features.hpp
//  AI1_Sokoban-solver_MM-TL
//
//  Created by Tobias Lundby on 19/10/16.
//

#pragma once

// Library include
#include <array>

// Class include
#include "Map.hpp" // Uses map and therefore needs to be included

// Defines
#define     NORTH   0; // NOTE: this goes opposite the y-axis
#define     EAST    1; // NOTE: this goes as the x-axis
#define     SOUTH   2; // NOTE: this goes as the y-axis
#define     WEST    3; // NOTE: this goes opposite the x-axis

// Namespaces
using namespace std;

class Sokoban_features
{
public:
	// feature_node
    struct feature_node
    {
        // Sokoban parameters
        vector< point2D > boxes; // vector for holding the different boxes
        point2D worker_pos;
        int worker_dir;

        // Tree parameters
		int depth;
        feature_node* parent = nullptr;
		vector< feature_node* > children; // vector for holding the children

		feature_node(feature_node* in_parent, int in_depth)
        : parent{ in_parent }, depth{ in_depth } { }
    };

	// Constructor, overload constructor, and destructor
	Sokoban_features();
	Sokoban_features(Map* map_ptr);
	~Sokoban_features();

	// Public variables

	// Public Methods
    feature_node* get_root_ptr();
	feature_node* insert_child(feature_node* parent_node);

	void print_node(feature_node* in_node);
	void solve();

private:
	// Private variables
    feature_node* root; // to hold the start sokoban features which is understod as the start placement of the elements / features
    Map* map;

	// Private Methods

};


Sokoban_features::Sokoban_features()
{
	root = nullptr;
	map = nullptr;
}

Sokoban_features::Sokoban_features(Map* map_ptr)
{
	root = nullptr;
	map = map_ptr;
}

Sokoban_features::~Sokoban_features()
{
	// Do cleanup
}

Sokoban_features::feature_node* Sokoban_features::get_root_ptr()
{
	return root;
}

Sokoban_features::feature_node* Sokoban_features::insert_child(feature_node* parent_node)
// Input: can either be the nullptr for creating the root of the tree or a pointer to the parent
// Output: Pointer to the created node / child
{
    feature_node* temp_node = nullptr;
    if (parent_node == nullptr) {
        if (root == nullptr) {
            temp_node = new Sokoban_features::feature_node{nullptr,0};
			temp_node->boxes = map->get_boxes();
			temp_node->worker_pos = map->get_worker();
			temp_node->worker_dir = NORTH;
            root = temp_node; // Set root as temp node after relevant info is saved from Map object
        } else
            cout << "Trying to create new root in existing tree, please create a new feature tree and try again" << endl;
    } else {
        temp_node = new Sokoban_features::feature_node{parent_node,parent_node->depth+1};
		temp_node->boxes = parent_node->boxes;
		temp_node->worker_pos = parent_node->worker_pos;
		temp_node->worker_dir = parent_node->worker_dir;
        parent_node->children.push_back(temp_node);
    }
	return temp_node;
}

void Sokoban_features::print_node(feature_node* in_node)
{
	map->print_map(in_node->worker_pos, in_node->boxes);
}

void Sokoban_features::solve()
{
	if (root == nullptr) {
		root = insert_child(nullptr); // Create tree root

		int new_nodes = 10000000;

		for (size_t i = 0; i < new_nodes; i++) {
			feature_node* tmp_node = insert_child(root);
			if (i == new_nodes -1) {
				tmp_node->worker_pos.x = 7;
				print_node(tmp_node);
			}
		}
	} else
		cout << "Tree already exists; break solve" << endl;
}
