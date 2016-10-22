//
//  MSokoban_features.hpp
//  AI1_Sokoban-solver_MM-TL
//
//  Created by Tobias Lundby on 19/10/16.
//

#pragma once

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
    // node
    struct node
    {
        // Sokoban parameters
        vector< array<int, 2> > obstacles; // vector for holding the different obstacles and then a 2x1 array inside to hold the x and y coordinate, respectively.
        array<int, 2> worker_pos;
        int worker_dir;

        // Tree parameters
        node *parent;
        int depth;
    };
	// Private variables
    node *root;
    Map *map;

	// Private Methods
};


Sokoban_features::Sokoban_features()
{
	root = NULL;
}

Sokoban_features::~Sokoban_features()
{
	// Do cleanup
}
