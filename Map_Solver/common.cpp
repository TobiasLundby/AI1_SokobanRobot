//
//  commom.cpp
//  AI1_Sokoban-solver_MM-TL
//
//  Created by Tobias Lundby on 26/09/16.
//  All files are licenced under the BSD 3-Clause (see LICENSE.md)
//
#pragma once

#define BF      0 // Breadth-first
#define Astar   1 // A*

#define F       1 // Forward move
#define B       2 // Backward move
#define L       3 // Turn left, CCW
#define R       4 // Turn right, CW
#define D       5 // Deploy move
#define A       6 // Approach move

struct point2D {
  int x;
  int y;
} ;
