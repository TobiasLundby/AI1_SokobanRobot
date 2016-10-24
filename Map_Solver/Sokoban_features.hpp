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


class Sokoban_features
{
public:
	// Constructor, overload constructor, and destructor
	Sokoban_features();
	~Sokoban_features();

	// Public variables

	// Public Methods

private:
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
	// Private variables
    feature_node* root; // to hold the start sokoban features which is understod as the start placement of the elements / features
    Map* map;

	// Private Methods
};


Sokoban_features::Sokoban_features()
{
	root = nullptr;
	map = nullptr;

	feature_node* t = new feature_node{nullptr,2};
	feature_node* s1 = new feature_node{t,t->depth+1};
	t->children.push_back(s1);

	feature_node* s2 = new feature_node{t,t->depth+1};
	t->children.push_back(s2);

	cout << "Depth of t is: " << t->depth << endl;
	cout << "Depth of s1 is: " << s1->depth << endl;
	cout << "Depth of s1's parent is: " << s1->parent->depth << endl;

	if (t->children.size() > 0)
		for (size_t i = 0; i < t->children.size(); i++)
			cout << "Depth of t's " << i+1 <<  " st child is: " << t->children.at(i)->depth << endl;
}

Sokoban_features::~Sokoban_features()
{
	// Do cleanup
}
