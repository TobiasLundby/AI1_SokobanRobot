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

#define     NORTH   = 0;
#define     EAST    = 1;
#define     SOUTH   = 2;
#define     WEST    = 3;

using namespace std;

class Sokoban_features
{
public:
	// feature_node
    struct feature_node
    {
        // Sokoban parameters
        vector< array<int, 2> > obstacles; // vector for holding the different obstacles and then a 2x1 array inside to hold the x and y coordinate, respectively.
        array<int, 2> worker_pos;
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
	~Sokoban_features();

	// Public variables

	// Public Methods
    feature_node* get_root_ptr();
	feature_node* insert_child(feature_node* parent_node);

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
            root = temp_node;
        } else
            cout << "Trying to create new root in existing tree, please create a new feature tree and try again" << endl;

    } else {
        temp_node = new Sokoban_features::feature_node{parent_node,parent_node->depth+1};
        parent_node->children.push_back(temp_node);
    }
	return temp_node;
}
