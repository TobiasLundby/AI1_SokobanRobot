//
//  MSokoban_features.hpp
//  AI1_Sokoban-solver_MM-TL
//
//  Created by Tobias Lundby on 19/10/16.
//  All files are licenced under the BSD 3-Clause (see LICENSE.md)
//

#pragma once

// Library include
#include <array>
#include <cmath>
#include <functional>

// Class include
#include "Map.hpp" // Uses map and therefore needs to be included
#include <time.h>       /* time */
#include <sys/time.h>       /* time */

// Defines
#define     NORTH   1 // NOTE: this goes opposite the y-axis
#define     EAST    2 // NOTE: this goes as the x-axis
#define     SOUTH   3 // NOTE: this goes as the y-axis
#define     WEST    4 // NOTE: this goes opposite the x-axis

// Moves
#define     forward     1
#define     backward    2
#define     left        3 // Turn CCW 90
#define     right       4 // Turn CW 90
#define     deploy      5
#define     approach    6

// Moves cost_to_node
#define     forward_cost     1
#define     backward_cost    2
#define     left_cost        1
#define     right_cost       1
#define     deploy_cost      2
#define     approach_cost    1.5

// Namespaces
using namespace std;

class Sokoban_features
{
public:
	// feature_node
    struct feature_node
    {
        // Sokoban parameters
        vector< point2D >   boxes; // vector for holding the different boxes
        vector< int >       box_goal_ref;
        point2D             worker_pos;
        int                 worker_dir;

        // Tree parameters
		int depth;
        double heuristic;
        double cost_to_node;

        feature_node* parent = nullptr;
		vector< feature_node* > children; // vector for holding the children
        vector< double > children_edge_cost; // vector for holding the children

		feature_node(feature_node* in_parent, int in_depth)
        : parent{ in_parent }, depth{ in_depth } { }
    };
    struct hash_node {
        unsigned long hash_value;
        feature_node* ref_node;
    };
    vector< hash_node > hash_table;
    vector< hash_node >* hash_table_ptr = &hash_table;

	// Constructor, overload constructor, and destructor
	Sokoban_features();
	Sokoban_features(Map* map_ptr);
	~Sokoban_features();

	// Public variables

	// Public Methods
    long long currentTimeUs();

    feature_node* get_root_ptr();
    feature_node* get_goal_node_ptr();
	feature_node* insert_child(feature_node* parent_node);

    void print_debug(const string& in_string);
    void print_info(const string& in_string);
	void print_node(feature_node* in_node);
	bool solve(int solver_type, int max_search);
    int  point_type(feature_node* in_node, point2D &inPoint, int map_type);
	int  point_type(feature_node* in_node, int in_x, int in_y, int map_type);
    double calcualte_heuristic(feature_node* in_node);
    double calculate_euclidian_distance(point2D &inPoint1, point2D &inPoint2);
    int calculate_taxicab_distance(point2D &inPoint1, point2D &inPoint2);
    void update_nearest_goals(feature_node* in_node);
    unsigned long hash_node_to_key(feature_node* in_node);
    bool nodes_match(feature_node* in_node1, feature_node* in_node2);
    bool update_parent_node(feature_node* &in_node_child, feature_node* in_node_new_parent);
	void print_branch_up(feature_node* in_node);
	bool remove_node(feature_node* &child);
    bool remove_only_node(feature_node* &child);
    bool remove_node_from_parent(feature_node* &child);
    bool add_node_to_parent(feature_node* &child, int pos); // Do not use this function since there is a bug with insert and doubles!
    bool update_node_to_parent(feature_node* &new_child, feature_node* &old_child);
    bool update_node_cost(feature_node* &child, double node_cost);
	bool goal_node(feature_node* in_node);
    bool goal_box(point2D in_box);
	bool move_box(feature_node* in_node, int in_x, int in_y, int offset_x, int offset_y);
    bool move_forward(feature_node* in_node);
    bool move_backward(feature_node* in_node);
	bool turn_right(feature_node* in_node);
	bool turn_left(feature_node* in_node);
    int  get_open_list_size();
    int  get_closed_list_size();

	// Hash table methods
    bool hash_table_insert(feature_node* &in_node, vector< hash_node >* hash_ptr);
    bool hash_table_insert(unsigned long in_hash_value, feature_node* &in_node, vector< hash_node >* hash_ptr);
    bool hash_table_exist(feature_node* &in_node, vector< hash_node >* hash_ptr);
    bool hash_table_exist(unsigned long in_hash_value, feature_node* &in_node, vector< hash_node >* hash_ptr);
    bool hash_table_delete(feature_node* &in_node, vector< hash_node >* hash_ptr);
    bool hash_table_delete(unsigned long in_hash_value, feature_node* &in_node, vector< hash_node >* hash_ptr);

private:
	// Private variables
    feature_node* root; // to hold the start sokoban features which is understod as the start placement of the elements / features
    feature_node* goal_ptr;
    Map* map;

    int chosen_graph_search;

	int peeked_notes = 0;

    vector< feature_node* > open_list; // Hold unvisited nodes
    vector< feature_node* > closed_list; // holds visited nodes

	// Private Methods
    hash<string> str_hash; // define a sting hashing func
};


Sokoban_features::Sokoban_features()
// Default constructor
{
	root = nullptr;
    goal_ptr = nullptr;
	map = nullptr;
}

Sokoban_features::Sokoban_features(Map* map_ptr)
// Overload constructor
{
	root = nullptr;
    goal_ptr = nullptr;
	map = map_ptr;
}

Sokoban_features::~Sokoban_features()
{
	// Do cleanup
}

long long Sokoban_features::currentTimeUs()
// Timer function
{
    timeval current;
    gettimeofday(&current, 0);
    return (long long)current.tv_sec * 1000000L + current.tv_usec;
}

Sokoban_features::feature_node* Sokoban_features::get_root_ptr()
// Returns the root node (ptr)
{
	return root;
}
Sokoban_features::feature_node* Sokoban_features::get_goal_node_ptr()
// Returns the goal node (ptr)
{
	return goal_ptr;
}

Sokoban_features::feature_node* Sokoban_features::insert_child(feature_node* parent_node)
// Input: can either be the nullptr for creating the root of the tree or a pointer to the parent
// Output: Pointer to the created node / child
{
	peeked_notes++;
    feature_node* temp_node = nullptr;
    if (parent_node == nullptr) {
        if (root == nullptr) {
            temp_node = new Sokoban_features::feature_node{nullptr,0};
			temp_node->boxes = map->get_boxes();
            for (size_t i = 0; i < temp_node->boxes.size(); i++) {
                temp_node->box_goal_ref.push_back(i);
            }
            if (chosen_graph_search == Astar) {
                update_nearest_goals(temp_node);
                temp_node->heuristic = calcualte_heuristic(temp_node);
            } else
                temp_node->heuristic = 0; // No heuristic for BF
			temp_node->worker_pos = map->get_worker();
			temp_node->worker_dir = NORTH;
            temp_node->cost_to_node = 0;

            root = temp_node; // Set root as temp node after relevant info is saved from Map object
        } else
            print_info("Trying to create new root in existing tree, please create a new feature tree and try again.");
    } else {
        temp_node = new Sokoban_features::feature_node{parent_node,parent_node->depth+1};
        // Save box information from parent
		temp_node->boxes = parent_node->boxes;
        temp_node->box_goal_ref = parent_node->box_goal_ref;
        // Save worker information from parent
		temp_node->worker_pos = parent_node->worker_pos;
		temp_node->worker_dir = parent_node->worker_dir;
        // Save stuff for searching
        temp_node->cost_to_node = parent_node->cost_to_node; // no movement yet so there is no added edge cost!
        if (chosen_graph_search == Astar) {
            update_nearest_goals(temp_node);
            temp_node->heuristic = calcualte_heuristic(temp_node);
        } else
            temp_node->heuristic = 0; // No heuristic for BF

        // Add new node to parent!
        parent_node->children.push_back(temp_node);
        parent_node->children_edge_cost.push_back(0); // no movement yet so there is no edge cost!
        // The node has no children at this stage
        // The node has no childrena and therefore no children edge cost
    }
	return temp_node;
}

bool Sokoban_features::remove_node(feature_node* &child)
// Removes the node and also from the parent
{
	feature_node* parent_node = child->parent;
	for (size_t i = 0; i < parent_node->children.size(); i++) {
		if (parent_node->children.at(i) == child) {
			parent_node->children.erase(parent_node->children.begin()+i);
			parent_node->children_edge_cost.erase(parent_node->children_edge_cost.begin()+i);
		}
	}
	delete child;
	child = nullptr;
	return true;
}

bool Sokoban_features::remove_only_node(feature_node* &child)
// Removes the node BUT NOT from the parent
{
	delete child;
	child = nullptr;
	return true;
}

bool Sokoban_features::remove_node_from_parent(feature_node* &child)
// Deletes the current child and from parent
{
	feature_node* parent_node = child->parent;
	for (size_t i = 0; i < parent_node->children.size(); i++) {
		if (parent_node->children.at(i) == child) {
			parent_node->children.erase(parent_node->children.begin()+i);
			parent_node->children_edge_cost.erase(parent_node->children_edge_cost.begin()+i);
		}
	}
	return true;
}
bool Sokoban_features::add_node_to_parent(feature_node* &child, int pos)
// Adds a node to the parent node
// Currently not used, replaced by the update_node_to_parent method which keeps track of the position as well
// Do not use this function since there is a bug with insert and doubles!!!
{
	feature_node* parent_node = child->parent;
    if (pos >= parent_node->children.size()) {
        parent_node->children.insert(parent_node->children.begin()+pos,child);
        //parent_node->children_edge_cost.insert(parent_node->children.begin()+pos,0);
        // NOTE no Astar here, just adds the edge_cost 0
    } else {
        parent_node->children.push_back(child);
        parent_node->children_edge_cost.push_back(child->cost_to_node);
    }
	return true;
}

bool Sokoban_features::update_node_to_parent(feature_node* &new_child, feature_node* &old_child) {
    feature_node* parent_node = old_child->parent;

	for (size_t i = 0; i < parent_node->children.size(); i++) {
		if (parent_node->children.at(i) == old_child) {
            parent_node->children.at(i) = new_child;
            parent_node->children_edge_cost.at(i) = new_child->cost_to_node;
		}
	}
	return true;
}

bool Sokoban_features::update_node_cost(feature_node* &child, double node_cost) {
    child->cost_to_node = child->cost_to_node + node_cost;
    feature_node* parent_node = child->parent;

	for (size_t i = 0; i < parent_node->children.size(); i++) {
		if (parent_node->children.at(i) == child) {
            parent_node->children_edge_cost.at(i) = node_cost;
            //cout << "Cost to node: " << child->cost_to_node << endl;
            //cout << "Node cost: " << parent_node->children_edge_cost.at(i) << endl;
		}
	}
	return true;
}

void Sokoban_features::print_node(feature_node* in_node)
// Prints a node using the Map class
{
	map->print_map(in_node->worker_pos, in_node->boxes, false);
}

bool Sokoban_features::solve(int solver_type, int max_search)
// Solver
// Input: BF or Astar (defines in common) and max search counter
// Output: true if a solution has been found
{
    /* initialize random seed: */
    srand (time(NULL));
    long long time_stamp = currentTimeUs();
    std::cout << "Time stamp is: " << time_stamp << std::endl;
    std::cout << "Time stamp diff is: " << currentTimeUs() - time_stamp << std::endl;

	if (root == nullptr) {
		if (solver_type == BF) {
            chosen_graph_search = BF;
			root = insert_child(nullptr); // Create tree root
			open_list.push_back(root);
            double branching = 0;
			while (open_list.size()) {
				feature_node* tmp_node = open_list.front();
				open_list.erase(open_list.begin());
                closed_list.push_back(tmp_node);
                // cout << "Parent" << endl;
                //print_node(tmp_node);
				move_forward(tmp_node);
                move_backward(tmp_node); // saves some moves but adds a lot of nodes (a factor more)
				turn_right(tmp_node);
				turn_left(tmp_node);
                //cout << "Children" << tmp_node->children.size() << endl;
                branching += tmp_node->children.size();
				bool break_search = false;
				for (size_t i = 0; i < tmp_node->children.size(); i++) {
                    //print_node(tmp_node->children.at(i));
					if (goal_node(tmp_node->children.at(i))) {
                        goal_ptr = tmp_node->children.at(i);
                        break_search = true;
                        branching /= closed_list.size();
                        cout << "Average branching is " << branching << endl;
						break;
					}
				}
				if (break_search)
					break;
                if (closed_list.size()%10000 == 0) {
                    print_info("Visited " + to_string(closed_list.size()) + " and " + to_string(open_list.size()) + " nodes waiting (peeked at " + to_string(peeked_notes) + " nodes)");
                }
                if (max_search <= closed_list.size()) {
                    return false;
                }
			}
		} else if (solver_type == Astar) {
            chosen_graph_search = Astar;
			root = insert_child(nullptr); // Create tree root
            open_list.push_back(root);
            double branching = 0;
			while (open_list.size()) {
                int smallest_Astar_id = 0;
                int smallest_Astar_value = open_list.at(0)->cost_to_node + open_list.at(0)->heuristic;
                for (size_t i = 1; i < open_list.size(); i++) {
                    if (open_list.at(i)->cost_to_node + open_list.at(i)->heuristic < smallest_Astar_value) {
                        smallest_Astar_id = i;
                    }
                }
                feature_node* tmp_node = open_list.at(smallest_Astar_id);
                //cout << "Chose id " << smallest_Astar_id << " which has fscore " << open_list.at(smallest_Astar_id)->cost_to_node + open_list.at(smallest_Astar_id)->heuristic << endl;

                open_list.erase(open_list.begin()+smallest_Astar_id);
                closed_list.push_back(tmp_node);

                move_forward(tmp_node);
                move_backward(tmp_node); // saves some moves but adds a lot of nodes (a factor more)
				turn_right(tmp_node);
				turn_left(tmp_node);

                branching += tmp_node->children.size();
				bool break_search = false;
				for (size_t i = 0; i < tmp_node->children.size(); i++) {
                    //print_node(tmp_node->children.at(i));
					if (goal_node(tmp_node->children.at(i))) {
                        goal_ptr = tmp_node->children.at(i);
                        break_search = true;
                        branching /= closed_list.size();
                        cout << "Average branching is " << branching << endl;
						break;
					}
				}
				if (break_search)
					break;
                if (closed_list.size()%10000 == 0) {
                    print_info("Visited " + to_string(closed_list.size()) + " and " + to_string(open_list.size()) + " nodes waiting (peeked at " + to_string(peeked_notes) + " nodes)");
                }
                if (max_search <= closed_list.size()) {
                    return false;
                }
            }
		} else {
			print_info("Unknown solver type, try again.");
		}
        if (open_list.size()) {
            return true;
        } else {
            return false;
        }
	} else {
        print_info("Tree already exists; breaking solver");
        return false;
    }
    return false; // default return!
}

bool Sokoban_features::move_forward(feature_node* in_node)
// Adds the forwards move node to the open list if it does NOT exist.
// If the node already exists the tree is manipulated if the new node has a smaller cost to node
{
    feature_node* tmp_node_child = insert_child(in_node);
    int move_x = 0;
    int move_y = 0;
    int deadlock_test_x1 = 0;
    int deadlock_test_x2 = 0;
    int deadlock_test_y1 = 0;
    int deadlock_test_y2 = 0;
    if (tmp_node_child->worker_dir == NORTH) {
        move_y = -1;
        deadlock_test_x1 = -1;
        deadlock_test_x2 = 1;
        deadlock_test_y1 = -2;
        deadlock_test_y2 = -2;
    } else if (tmp_node_child->worker_dir == EAST) {
        move_x = 1;
        deadlock_test_x1 = 2;
        deadlock_test_x2 = 2;
        deadlock_test_y1 = -1;
        deadlock_test_y2 = 1;
    } else if (tmp_node_child->worker_dir == SOUTH) {
        move_y = 1;
        deadlock_test_x1 = 1;
        deadlock_test_x2 = -1;
        deadlock_test_y1 = 2;
        deadlock_test_y2 = 2;
    } else if (tmp_node_child->worker_dir == WEST) {
        move_x = -1;
        deadlock_test_x1 = -2;
        deadlock_test_x2 = -2;
        deadlock_test_y1 = 1;
        deadlock_test_y2 = -1;
    }
    // First test if there is free space to move forward
    // second test if there is a box in front and if there is test for freespace or goal in front of box
    if (point_type(tmp_node_child, tmp_node_child->worker_pos.x + move_x, tmp_node_child->worker_pos.y + move_y, worker) == freespace
	or point_type(tmp_node_child, tmp_node_child->worker_pos.x + move_x, tmp_node_child->worker_pos.y + move_y, worker) == goal) {
        // MOVE FORWARD TO FREESPACE
        //tmp_node_child->cost_to_node = tmp_node_child->cost_to_node + 1*forward_cost;
        update_node_cost(tmp_node_child, 1*forward_cost);
        tmp_node_child->worker_pos.x = tmp_node_child->worker_pos.x + move_x;
        tmp_node_child->worker_pos.y = tmp_node_child->worker_pos.y + move_y;

        feature_node* tmp_node_child_for_check = tmp_node_child;
        if (hash_table_insert(tmp_node_child_for_check, hash_table_ptr)) {
            open_list.push_back(tmp_node_child);
            return true;
        } else {
            // Check if the returned
            if (tmp_node_child->cost_to_node < tmp_node_child_for_check->cost_to_node) {
                tmp_node_child_for_check->cost_to_node = tmp_node_child->cost_to_node;
                remove_node_from_parent(tmp_node_child_for_check);
                tmp_node_child_for_check->parent = tmp_node_child->parent;
                //add_node_to_parent(tmp_node_child_for_check,0);
                update_node_to_parent(tmp_node_child_for_check, tmp_node_child);
                remove_only_node(tmp_node_child);
                return true;
            } else {
                remove_node(tmp_node_child);
                return false;
            }
        }
        return false;
    } else if (point_type(tmp_node_child, tmp_node_child->worker_pos.x + move_x, tmp_node_child->worker_pos.y + move_y, worker) == box
                and (point_type(tmp_node_child, tmp_node_child->worker_pos.x + move_x*2, tmp_node_child->worker_pos.y + move_y*2, worker) == goal
                    or point_type(tmp_node_child, tmp_node_child->worker_pos.x + move_x*2, tmp_node_child->worker_pos.y + move_y*2, box) == freespace) ) {

        //tmp_node_child->cost_to_node = tmp_node_child->cost_to_node + approach_cost;
        update_node_cost(tmp_node_child, 1*approach_cost);
        // PUSH MOVE
        move_box(tmp_node_child,tmp_node_child->worker_pos.x + move_x,tmp_node_child->worker_pos.y + move_y,move_x,move_y);
        tmp_node_child->worker_pos.x = tmp_node_child->worker_pos.x + move_x;
        tmp_node_child->worker_pos.y = tmp_node_child->worker_pos.y + move_y;

        feature_node* tmp_node_child_for_check = tmp_node_child;
        if (hash_table_insert(tmp_node_child_for_check, hash_table_ptr)) {
            point2D tmp_point;
            tmp_point.x = tmp_node_child->worker_pos.x + move_x;
            tmp_point.y = tmp_node_child->worker_pos.y + move_y;
            open_list.push_back(tmp_node_child);
            return true;
        } else {
            if (tmp_node_child->cost_to_node < tmp_node_child_for_check->cost_to_node) {
                tmp_node_child_for_check->cost_to_node = tmp_node_child->cost_to_node;
                remove_node_from_parent(tmp_node_child_for_check);
                tmp_node_child_for_check->parent = tmp_node_child->parent;
                //add_node_to_parent(tmp_node_child_for_check,0);
                update_node_to_parent(tmp_node_child_for_check, tmp_node_child);
                remove_only_node(tmp_node_child);
                return true;
            } else {
                remove_node(tmp_node_child);
                return false;
            }
        }
    } else {
        remove_node(tmp_node_child);
        return false;
    }
    return false;
}
bool Sokoban_features::move_backward(feature_node* in_node)
// Adds the backwards move node to the open list if it does NOT exist.
// If the node already exists the tree is manipulated if the new node has a smaller cost to node
{
    feature_node* tmp_node_child = insert_child(in_node);
    int move_x = 0;
    int move_y = 0;
    if (tmp_node_child->worker_dir == NORTH) {
        move_y = 1;
    } else if (tmp_node_child->worker_dir == EAST) {
        move_x = -1;
    } else if (tmp_node_child->worker_dir == SOUTH) {
        move_y = -1;
    } else if (tmp_node_child->worker_dir == WEST) {
        move_x = 1;
    }

    // First test if there is free space to move forward
    // second test if there is a box in front and if there is test for freespace or goal in front of box
    if (point_type(tmp_node_child, tmp_node_child->worker_pos.x + move_x, tmp_node_child->worker_pos.y + move_y, worker) == freespace
	or point_type(tmp_node_child, tmp_node_child->worker_pos.x + move_x, tmp_node_child->worker_pos.y + move_y, worker) == goal) {
        // MOVE FORWARD TO FREESPACE
        //tmp_node_child->cost_to_node = tmp_node_child->cost_to_node + 1*backward_cost;
        update_node_cost(tmp_node_child, 1*backward_cost);
        tmp_node_child->worker_pos.x = tmp_node_child->worker_pos.x + move_x;
        tmp_node_child->worker_pos.y = tmp_node_child->worker_pos.y + move_y;
        //print_node(tmp_node_child);

        feature_node* tmp_node_child_for_check = tmp_node_child;
        if (hash_table_insert(tmp_node_child_for_check, hash_table_ptr)) {
            open_list.push_back(tmp_node_child);
            return true;
        } else {
            // Check if the returned
            if (tmp_node_child->cost_to_node < tmp_node_child_for_check->cost_to_node) {
                tmp_node_child_for_check->cost_to_node = tmp_node_child->cost_to_node;
                remove_node_from_parent(tmp_node_child_for_check);
                tmp_node_child_for_check->parent = tmp_node_child->parent;
                //add_node_to_parent(tmp_node_child_for_check,0);
                update_node_to_parent(tmp_node_child_for_check, tmp_node_child);
                remove_only_node(tmp_node_child);
                return true;
            } else {
                remove_node(tmp_node_child);
                return false;
            }
        }
        return false;
    } else {
        remove_node(tmp_node_child);
        return false;
    }
    return false;
}
bool Sokoban_features::turn_right(feature_node* in_node)
// Adds the right turn node to the open list if it does NOT exist.
// If the node already exists the tree is manipulated if the new node has a smaller cost to node
{
	// Right CW
	feature_node* tmp_node_child_cw = insert_child(in_node);
	// Test for deploy NOTE
	//tmp_node_child_cw->cost_to_node = tmp_node_child_cw->cost_to_node + 1*left_cost;
    update_node_cost(tmp_node_child_cw, 1*left_cost);
	if (tmp_node_child_cw->worker_dir >= WEST) {
		tmp_node_child_cw->worker_dir = NORTH;
	} else {
		tmp_node_child_cw->worker_dir += 1;
	}
	feature_node* tmp_node_child_for_check_cw = tmp_node_child_cw;
	if (hash_table_insert(tmp_node_child_for_check_cw, hash_table_ptr)) {
		open_list.push_back(tmp_node_child_cw);
		return true;
	} else {
		if (tmp_node_child_cw->cost_to_node < tmp_node_child_for_check_cw->cost_to_node) {
			tmp_node_child_for_check_cw->cost_to_node = tmp_node_child_cw->cost_to_node;
			remove_node_from_parent(tmp_node_child_for_check_cw);
			tmp_node_child_for_check_cw->parent = tmp_node_child_cw->parent;
			//add_node_to_parent(tmp_node_child_for_check_cw,0);
            update_node_to_parent(tmp_node_child_for_check_cw, tmp_node_child_cw);
			remove_only_node(tmp_node_child_cw);
			return true;
		} else {
			remove_node(tmp_node_child_cw);
			return false;
		}
	}
}
bool Sokoban_features::turn_left(feature_node* in_node)
// Adds the left turn node to the open list if it does NOT exist.
// If the node already exists the tree is manipulated if the new node has a smaller cost to node
{
	// Left CCW
	feature_node* tmp_node_child_ccw = insert_child(in_node);
	// test for deploy NOTE
	//tmp_node_child_ccw->cost_to_node = tmp_node_child_ccw->cost_to_node + 1*left_cost;
    update_node_cost(tmp_node_child_ccw, 1*left_cost);
	if (tmp_node_child_ccw->worker_dir <= NORTH) {
		tmp_node_child_ccw->worker_dir = WEST;
	} else {
		tmp_node_child_ccw->worker_dir -= 1;
	}
	feature_node* tmp_node_child_for_check_ccw = tmp_node_child_ccw;
	if (hash_table_insert(tmp_node_child_for_check_ccw, hash_table_ptr)) {
		open_list.push_back(tmp_node_child_ccw);
		return true;
	} else {
		if (tmp_node_child_ccw->cost_to_node < tmp_node_child_for_check_ccw->cost_to_node) {
			tmp_node_child_for_check_ccw->cost_to_node = tmp_node_child_ccw->cost_to_node;
			remove_node_from_parent(tmp_node_child_for_check_ccw);
			tmp_node_child_for_check_ccw->parent = tmp_node_child_ccw->parent;
			//add_node_to_parent(tmp_node_child_for_check_ccw,0);
            update_node_to_parent(tmp_node_child_for_check_ccw, tmp_node_child_ccw);
			remove_only_node(tmp_node_child_ccw);
			return true;
		} else {
			remove_node(tmp_node_child_ccw);
			return false;
		}
	}
}

int  Sokoban_features::point_type(feature_node* in_node, int in_x, int in_y, int map_type)
// An overload function for the point_type; makes a point from the input positions
{
	point2D tmp_point;
	tmp_point.x = in_x;
	tmp_point.y = in_y;
	return point_type(in_node,tmp_point, map_type);
}
int  Sokoban_features::point_type(feature_node* in_node, point2D &inPoint, int map_type)
// Returns the type of the point; the actual return value is given by defines
{
	if ( (inPoint.x >= 0 and inPoint.x < map->get_width()) and (inPoint.y >= 0 and inPoint.y < map->get_height()) ) {
		if (in_node->worker_pos.x == inPoint.x and in_node->worker_pos.y == inPoint.y)
	        return worker;
	    for (size_t i = 0; i < in_node->boxes.size(); i++)
	        if (in_node->boxes.at(i).x == inPoint.x and in_node->boxes.at(i).y == inPoint.y)
	            return box;
	    return map->map_point_type(inPoint,map_type);
	}
    return undefined;
}

double Sokoban_features::calcualte_heuristic(feature_node* in_node)
// Calculates and returns the heuristic for the input node.
{
    // Still no heuristic!!
    int heuristic = 0; // integer due to the taxicap distance from the wavefront!
    for (size_t i = 0; i < in_node->boxes.size(); i++) {
        //cout << "Box " << i << " and goal " << in_node->box_goal_ref.at(i) << " has heuristic " << map->wavefront_distance(in_node->boxes.at(i), in_node->box_goal_ref.at(i)) << endl;
        heuristic += map->wavefront_distance(in_node->boxes.at(i), in_node->box_goal_ref.at(i));
    }
    //cout << "Cost to node " << in_node->cost_to_node << endl;
    //cout << "A* value " << in_node->cost_to_node+heuristic << endl;
    return heuristic;
}

double Sokoban_features::calculate_euclidian_distance(point2D &inPoint1, point2D &inPoint2)
// Calculates the euclidian distance between point1 and point2 and returns the distance.
{
    return sqrt(pow(inPoint1.x-inPoint2.x,2)+pow(inPoint1.y-inPoint2.y,2));
}

int Sokoban_features::calculate_taxicab_distance(point2D &inPoint1, point2D &inPoint2)
// Calculates the taxicab distance between point1 and point2 and returns the distance.
{
    return abs(inPoint1.x-inPoint2.x)+abs(inPoint1.y-inPoint2.y);
}

void Sokoban_features::update_nearest_goals(feature_node* in_node)
// Each node contains a box_goal_ref which is a vector where each of the boxes are associated with a goal
// This method updates these associations which in term can be used for a heuristic
// Currently it is just using the euclidian distance which is not a good distance, a better one would be a wavefront map for each of the goals
{
    bool debug_update_nearest_goals = false;
    //in_node->box_goal_ref.clear();
    for (size_t i = 0; i < in_node->box_goal_ref.size(); i++) {
        in_node->box_goal_ref.at(i) = -1;
    }
    bool missing_links = true;
    while (missing_links) {
        int tmp_id;
        for (size_t i = 0; i < in_node->box_goal_ref.size(); i++) {
            if (in_node->box_goal_ref.at(i) == -1) {
                tmp_id = i;
                break;
            }
        }

        double tmp_distance1 = 999999999;
        double tmp_distance2 = 999999999;
        int goal_id = -1;
        bool tmp_goal_free;
        int  goal_taken_by = -1;
        if (debug_update_nearest_goals)
            cout << "working on " << tmp_id << " pos " << in_node->boxes.at(tmp_id).x << ", " << in_node->boxes.at(tmp_id).y << endl;
        for (size_t j = 0; j < in_node->boxes.size(); j++) {
            tmp_distance2 = calculate_euclidian_distance(in_node->boxes.at(tmp_id),map->get_goals().at(j));
            if (debug_update_nearest_goals)
                cout << "calculated distance to goal " << j << ": " << tmp_distance2 << endl;
            if (tmp_distance2 < tmp_distance1) {
                tmp_goal_free = true;
                for (size_t k = 0; k < in_node->box_goal_ref.size(); k++) {
                    if (in_node->box_goal_ref.at(k) == j) {
                        tmp_goal_free = false;
                        goal_taken_by = k;
                        if (debug_update_nearest_goals)
                            cout << "goal " << j << " is taken by " << k << endl;
                    }
                }
                if (tmp_goal_free) {
                    goal_id = j;
                    tmp_distance1 = tmp_distance2;
                } else {
                    if (calculate_euclidian_distance(in_node->boxes.at(tmp_id),map->get_goals().at(j)) < calculate_euclidian_distance(in_node->boxes.at(goal_taken_by), map->get_goals().at(in_node->box_goal_ref.at(goal_taken_by)))) {
                        goal_id = j;
                        tmp_distance1 = tmp_distance2;
                    } else {
                        if (debug_update_nearest_goals)
                            cout << "cannot take goal " << j  << " with distance " << calculate_euclidian_distance(in_node->boxes.at(goal_taken_by), map->get_goals().at(in_node->box_goal_ref.at(goal_taken_by))) << endl;
                        tmp_goal_free = true;
                        goal_taken_by = -1;
                    }
                }
            }
        }

        in_node->box_goal_ref.at(tmp_id) = goal_id;
        if (debug_update_nearest_goals)
            cout << "goal pushed " << goal_id << " pos " << map->get_goals().at(goal_id).x << ", " << map->get_goals().at(goal_id).y << endl << endl;
        for (size_t k = 0; k < in_node->box_goal_ref.size(); k++) {
            if (in_node->box_goal_ref.at(k) == goal_id and k != tmp_id) {
                if (debug_update_nearest_goals)
                    cout << "Clearing " << k << endl << endl;
                in_node->box_goal_ref.at(k) = -1;
            }
        }

        missing_links = false;
        for (size_t i = 0; i < in_node->box_goal_ref.size(); i++) {
            if (in_node->box_goal_ref.at(i) == -1) {
                missing_links = true;
                break;
            }
        }
    }
}

unsigned long Sokoban_features::hash_node_to_key(feature_node* in_node)
// Hashes a node
// the boxes are not treated as unique
{
    vector< int > tmp_vec;
    for (size_t i = 0; i < in_node->boxes.size(); i++) {
        tmp_vec.push_back((in_node->boxes.at(i).x * 10) + in_node->boxes.at(i).y);
    }
    sort(tmp_vec.begin(), tmp_vec.end());

    string tmp_str = "";
    tmp_str += to_string(in_node->worker_pos.x);
    tmp_str += to_string(in_node->worker_pos.y);
    tmp_str += to_string(in_node->worker_dir);

    for (size_t i = 0; i < tmp_vec.size(); i++) {
        tmp_str += to_string(tmp_vec.at(i));
    }

    return str_hash(tmp_str);
}

bool Sokoban_features::nodes_match(feature_node* in_node1, feature_node* in_node2)
// Hashes each node and compares the hashing value; note that the boxes are not treated as unique
{
    return hash_node_to_key(in_node1) == hash_node_to_key(in_node2);
}

bool Sokoban_features::update_parent_node(feature_node* &in_node_child, feature_node* in_node_new_parent)
// Updates the parent node of the first input node with the parent node of the second input node.
{
    if ( (in_node_child->parent = in_node_new_parent) ) {
        return true;
    }
    return false;
}

void Sokoban_features::print_debug(const string& in_string)
// A method for nicer debug messages
{
    cout << "[DEBUG] " << in_string << endl;
}
void Sokoban_features::print_info(const string& in_string)
// A method for nicer info messages
{
    cout << "[INFO] " << in_string << endl;
}

void Sokoban_features::print_branch_up(feature_node* in_node)
// Prints the branch in decreasing order so starting from the root.
{
	cout << endl;
	print_info(" *** PRINTING BRANCH - OUTPUT LENGTH DEPENDS ON BRANCH DEPTH ***");
	feature_node* tmp_node = in_node;

    vector< feature_node* > branch;
    while (tmp_node != nullptr) {
		branch.push_back(tmp_node);
		tmp_node = tmp_node->parent;
	}
	while (branch.size()) {
        tmp_node = branch.back();
        branch.pop_back();
		print_info("Depth is " + to_string(tmp_node->depth));
        print_info("Cost to node is " + to_string(tmp_node->cost_to_node));
        print_info("Worker has direction: "+to_string(tmp_node->worker_dir));
		print_node(tmp_node);
	}
}

bool Sokoban_features::goal_node(feature_node* in_node)
// Tests whether or not the input node is a goal node; a goal node is a node where all the boxes are at the goals (no specific order nessecary)
{
	vector< int > tmp_vec_box;
    for (size_t i = 0; i < in_node->boxes.size(); i++) {
        tmp_vec_box.push_back((in_node->boxes.at(i).x * 10) + in_node->boxes.at(i).y);
    }

	vector< point2D > goals = map->get_goals();
	vector< int > tmp_vec_goal;
    for (size_t i = 0; i < goals.size(); i++) {
        tmp_vec_goal.push_back((goals.at(i).x * 10) + goals.at(i).y);
    }

	bool equal = true;
    int counter = 0;
	for (size_t i = 0; i < tmp_vec_box.size(); i++) {
        for (size_t j = 0; j < tmp_vec_goal.size(); j++) {
            if (tmp_vec_box.at(i) == tmp_vec_goal.at(j)) {
                counter++;
            }
        }
	}
    if (counter == tmp_vec_box.size()) {
        return true;
    }
    return false;
}
bool Sokoban_features::goal_box(point2D in_box)
// Tests if the input box is at a goal
{
    vector< point2D > goals = map->get_goals();

    for (size_t i = 0; i < goals.size(); i++) {
        if (goals.at(i).x == in_box.x and goals.at(i).y == in_box.y) {
            return true;
        }
    }
    return false;
}

bool Sokoban_features::move_box(feature_node* in_node, int in_x, int in_y, int offset_x, int offset_y)
// Moves a box given from the position of the box and moves it the given amount by the offset inputs.
{
	for (size_t i = 0; i < in_node->boxes.size(); i++) {
		if (in_node->boxes.at(i).x == in_x and in_node->boxes.at(i).y == in_y) {
			in_node->boxes.at(i).x = in_node->boxes.at(i).x + offset_x;
			in_node->boxes.at(i).y = in_node->boxes.at(i).y + offset_y;
			return true;
		}
	}
	return false;
}

int  Sokoban_features::get_open_list_size()
// Returns the open list
{
    return open_list.size();
}
int  Sokoban_features::get_closed_list_size()
// Returns the closed list
{
    return closed_list.size();
}

// Hash table methods **********************************************************
bool Sokoban_features::hash_table_insert(feature_node* &in_node, vector< hash_node >* hash_ptr)
// An overload function for the hash_table_insert; hashes the input node
{
    return hash_table_insert(hash_node_to_key(in_node), in_node, hash_ptr);
}
bool Sokoban_features::hash_table_insert(unsigned long in_hash_value, feature_node* &in_node, vector< hash_node >* hash_ptr)
// inserts with a time constant of log(n) where n = hash_table_size
// if the element exists the in_node is changed to the existing element so it can be used for futher processing
// return true if element is inserted and false if it already exists
{
    hash_node tmp_hash_node;
    tmp_hash_node.hash_value = in_hash_value;
    tmp_hash_node.ref_node = in_node;

    if (hash_ptr->size() > 0) { // test if has_table is empty
        int const start_hash_table = 0; // access this element as hash_ptr->begin()
        int start_itr = start_hash_table;
        int const end_hash_table = hash_ptr->size()-1; // access this element as hash_ptr->end()
        int end_itr = end_hash_table;

        if (start_itr == end_itr) { // test if has_table only consists of 1 element
            if (in_hash_value == hash_ptr->at(start_itr).hash_value) {
                // element exists return ptr. NOTE
                in_node = hash_ptr->at(start_itr).ref_node;
                return false;
            } else if (in_hash_value > hash_ptr->at(start_itr).hash_value) {
                hash_ptr->push_back(tmp_hash_node);
                return true; // node inserted after 1st element
            } else {
                hash_ptr->insert(hash_ptr->begin()+start_itr,tmp_hash_node);
                return true; // node inserted before 1st element
            }
        }

        //cout << "[DEBUG INFO: hash_table_insert] entering while loop for " << in_hash_value << endl;
        int tmp_itr;
        while (true) {
            tmp_itr = (end_itr-start_itr)/2;
            //cout << "[DEBUG INFO: hash_table_insert] tmp_itr has value " << tmp_itr << ", start " << start_itr << ", end " << end_itr << endl; // debug info

            if (tmp_itr == 0) {
                if (in_hash_value == hash_ptr->at(start_itr).hash_value) {
                    // element exists return ptr. NOTE
                    in_node = hash_ptr->at(start_itr).ref_node;
                    return false;
                } else if (in_hash_value > hash_ptr->at(start_itr).hash_value) {
                    if (in_hash_value == hash_ptr->at(end_itr).hash_value) {
                        // element exists return ptr. NOTE
                        in_node = hash_ptr->at(end_itr).ref_node;
                        return false;
                    } else if (in_hash_value > hash_ptr->at(end_itr).hash_value) {
                        if (end_itr == end_hash_table) {
                            hash_ptr->push_back(tmp_hash_node);
                            return true; // node inserted at end of list
                        } else {
                            hash_ptr->insert(hash_ptr->begin() + end_itr + 1,tmp_hash_node);
                            return true; // node inserted after end_ptr
                        }
                    } else {
                        hash_ptr->insert(hash_ptr->begin() + end_itr,tmp_hash_node);
                        return true; // node inserted between ptr's
                    }
                } else {
                    hash_ptr->insert(hash_ptr->begin() + start_itr,tmp_hash_node); // insert at start itr space
                    return true; // node inserted before 1st element
                }
            } else {
                if (in_hash_value == hash_ptr->at(start_itr+tmp_itr).hash_value) {
                    // element exists return ptr.
                    in_node = hash_ptr->at(start_itr+tmp_itr).ref_node;
                    return false;
                } else if (in_hash_value > hash_ptr->at(start_itr+tmp_itr).hash_value) {
                    start_itr += tmp_itr;
                } else {
                    end_itr -= tmp_itr;
                }
            }
        }
    } else {
        hash_ptr->push_back(tmp_hash_node);
        return true; // first element inserted
    }
    return true;
}

bool Sokoban_features::hash_table_exist(feature_node* &in_node, vector< hash_node >* hash_ptr)
// An overload function for the hash_table_exist; hashes the input node
{
    //cout << "[DEBUG: hashing] Hash key " << hash_node_to_key(in_node) << endl;
    return hash_table_exist(hash_node_to_key(in_node), in_node, hash_ptr);
}
bool Sokoban_features::hash_table_exist(unsigned long in_hash_value, feature_node* &in_node, vector< hash_node >* hash_ptr)
// Searches for a element in the the hash table and if it exists the pointer to the found object is passes back in in_node and it returns true; otherwise no pointer return and false return value
{
    if (hash_ptr->size() > 0) { // test if has_table is empty
        int const start_hash_table = 0; // access this element as hash_ptr->begin()
        int start_itr = start_hash_table;
        int const end_hash_table = hash_ptr->size()-1; // access this element as hash_ptr->end()
        int end_itr = end_hash_table;

        int tmp_itr;
        while (true) {
            tmp_itr = (end_itr-start_itr)/2;
            //cout << "[DEBUG INFO: hash_table_insert] tmp_itr has value " << tmp_itr << ", start " << start_itr << ", end " << end_itr << endl; // debug info

            if (tmp_itr == 0) {
                if (in_hash_value == hash_ptr->at(start_itr).hash_value) {
                    in_node = hash_ptr->at(start_itr).ref_node;
                    return true;
                } else if (in_hash_value == hash_ptr->at(end_itr).hash_value) {
                    in_node = hash_ptr->at(end_itr).ref_node;
                    return true;
                } else {
                    return false;
                }
            } else {
                if (in_hash_value == hash_ptr->at(start_itr+tmp_itr).hash_value) {
                    in_node = hash_ptr->at(start_itr+tmp_itr).ref_node;
                    return true;
                } else if (in_hash_value > hash_ptr->at(start_itr+tmp_itr).hash_value) {
                    start_itr += tmp_itr;
                } else {
                    end_itr -= tmp_itr;
                }
            }
        }
    } else {
        return false;
    }
}

bool Sokoban_features::hash_table_delete(feature_node* &in_node, vector< hash_node >* hash_ptr)
// An overload function for the hash_table_delete; hashes the input node
{
    return hash_table_delete(hash_node_to_key(in_node), in_node, hash_ptr);
}
bool Sokoban_features::hash_table_delete(unsigned long in_hash_value, feature_node* &in_node, vector< hash_node >* hash_ptr)
// Deletes an element in the list if it exists (return value true)
{
    if (hash_ptr->size() > 0) { // test if has_table is empty
        int const start_hash_table = 0; // access this element as hash_ptr->begin()
        int start_itr = start_hash_table;
        int const end_hash_table = hash_ptr->size()-1; // access this element as hash_ptr->end()
        int end_itr = end_hash_table;

        int tmp_itr;
        while (true) {
            tmp_itr = (end_itr-start_itr)/2;
            //cout << "[DEBUG INFO: hash_table_insert] tmp_itr has value " << tmp_itr << ", start " << start_itr << ", end " << end_itr << endl; // debug info

            if (tmp_itr == 0) {
                if (in_hash_value == hash_ptr->at(start_itr).hash_value) {
                    hash_ptr->erase(hash_ptr->begin() + start_itr);
                    return true;
                } else if (in_hash_value == hash_ptr->at(end_itr).hash_value) {
                    hash_ptr->erase(hash_ptr->begin() + end_itr);
                    return true;
                } else {
                    return false;
                }
            } else {
                if (in_hash_value == hash_ptr->at(start_itr+tmp_itr).hash_value) {
                    hash_ptr->erase(hash_ptr->begin() + start_itr+tmp_itr);
                    return true;
                } else if (in_hash_value > hash_ptr->at(start_itr+tmp_itr).hash_value) {
                    start_itr += tmp_itr;
                } else {
                    end_itr -= tmp_itr;
                }
            }
        }
    } else {
        return false;
    }
}
