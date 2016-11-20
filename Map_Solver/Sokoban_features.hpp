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

// Defines
#define     NORTH   1; // NOTE: this goes opposite the y-axis
#define     EAST    2; // NOTE: this goes as the x-axis
#define     SOUTH   3; // NOTE: this goes as the y-axis
#define     WEST    4; // NOTE: this goes opposite the x-axis

// Moves
#define     forward     1;
#define     backward    2;
#define     left        3; // Turn CCW 90
#define     right       4; // Turn CW 90
#define     deploy      5;
#define     approach    6;

// Moves cost_to_node
#define     forward_cost     1;
#define     backward_cost    1;
#define     left_cost        1;
#define     right_cost       1;
#define     deploy_cost      2;
#define     approach_cost    1.5;

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
        vector< feature_node* > children_edge_cost; // vector for holding the children

		feature_node(feature_node* in_parent, int in_depth)
        : parent{ in_parent }, depth{ in_depth } { }
    };
    struct hash_node {
        unsigned long hash_value;
        feature_node* ref_node;
    };
    vector< hash_node > hash_table;

	// Constructor, overload constructor, and destructor
	Sokoban_features();
	Sokoban_features(Map* map_ptr);
	~Sokoban_features();

	// Public variables

	// Public Methods
    feature_node* get_root_ptr();
	feature_node* insert_child(feature_node* parent_node);

    void print_debug(const string& in_string);
    void print_info(const string& in_string);
	void print_node(feature_node* in_node);
	void solve();
    int  point_type(feature_node* in_node, point2D &inPoint);
    double calcualte_heuristic(feature_node* in_node);
    double calculate_euclidian_distance(point2D &inPoint1, point2D &inPoint2);
    int calculate_taxicab_distance(point2D &inPoint1, point2D &inPoint2);
    void update_nearest_goals(feature_node* in_node);
    unsigned long hash_node_to_key(feature_node* in_node);
    bool nodes_match(feature_node* in_node1, feature_node* in_node2);
    bool update_parent_node(feature_node* &in_node_child, feature_node* in_node_new_parent);

    bool hash_table_insert(feature_node* &in_node);
    bool hash_table_insert(unsigned long in_hash_value, feature_node* &in_node);
    bool hash_table_exist(feature_node* &in_node);
    bool hash_table_exist(unsigned long in_hash_value, feature_node* &in_node);
    bool hash_table_delete(feature_node* &in_node);
    bool hash_table_delete(unsigned long in_hash_value, feature_node* &in_node);

private:
	// Private variables
    feature_node* root; // to hold the start sokoban features which is understod as the start placement of the elements / features
    Map* map;

    vector< feature_node* > open_list; // Hold unvisited nodes
    vector< feature_node* > closed_list; // holds visited nodes

	// Private Methods
    hash<string> str_hash; // define a sting hashing func
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
            for (size_t i = 0; i < temp_node->boxes.size(); i++) {
                temp_node->box_goal_ref.push_back(i);
            }
            update_nearest_goals(temp_node);
			temp_node->worker_pos = map->get_worker();
			temp_node->worker_dir = NORTH;
            temp_node->cost_to_node = 0;
            temp_node->heuristic = 0; // No heuristic!

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
        temp_node->heuristic = 0; // No heuristic!
        // Add new node to parent!
        parent_node->children.push_back(temp_node);
        parent_node->children_edge_cost.push_back(0); // no movement yet so there is no edge cost!
        // The node has no children at this stage
        // The node has no childrena and therefore no children edge cost
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

		int new_nodes = 10;

		for (size_t i = 0; i < new_nodes; i++) {
			feature_node* tmp_node = insert_child(root);
			// if (i == new_nodes -1) {
			// 	tmp_node->worker_pos.x = 7;
			// 	print_node(tmp_node);
			// }
		}
	} else
        print_info("Tree already exists; breaking solver");

    // print_info("Point type test");
    // point2D point_tmp;
    // point_tmp.x = 2;
    // point_tmp.y = 3;
    // cout << point_type(root,point_tmp) << endl;
    //
    //print_info("Nearest goals test");
    // update_nearest_goals(root);
    // for (size_t i = 0; i < root->boxes.size(); i++)
    //     cout << "Box " << i << " has goal " << root->box_goal_ref.at(i) << " as nearest goal" << endl;
    //
    //print_info("Hashing test");
    // feature_node* tmp_node = insert_child(root);
    // cout << hash_table_insert(tmp_node) << " node " << tmp_node << endl;
    // for (size_t i = 0; i < hash_table.size(); i++)
    //     cout << "element " << i << " contains " << hash_table.at(i).hash_value << " and " << hash_table.at(i).ref_node << endl;

}

int  Sokoban_features::point_type(feature_node* in_node, point2D &inPoint)
{
    if (in_node->worker_pos.x == inPoint.x and in_node->worker_pos.y == inPoint.y)
        return worker;
    for (size_t i = 0; i < in_node->boxes.size(); i++)
        if (in_node->boxes.at(i).x == inPoint.x and in_node->boxes.at(i).y == inPoint.y)
            return box;
    return map->map_point_type(inPoint);
}

double Sokoban_features::calcualte_heuristic(feature_node* in_node)
{
    // Still no heuristic!!
    return 0;
}

double Sokoban_features::calculate_euclidian_distance(point2D &inPoint1, point2D &inPoint2)
{
    return sqrt(pow(inPoint1.x-inPoint2.x,2)+pow(inPoint1.y-inPoint2.y,2));
}

int Sokoban_features::calculate_taxicab_distance(point2D &inPoint1, point2D &inPoint2)
{
    return abs(inPoint1.x-inPoint2.x)+abs(inPoint1.y-inPoint2.y);
}

void Sokoban_features::update_nearest_goals(feature_node* in_node)
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
{
    if ( (in_node_child->parent = in_node_new_parent) ) {
        return true;
    }
    return false;
}

void Sokoban_features::print_debug(const string& in_string)
{
    cout << "[DEBUG] " << in_string << endl;
}
void Sokoban_features::print_info(const string& in_string)
{
    cout << "[INFO] " << in_string << endl;
}


// Hash table methods **********************************************************
bool Sokoban_features::hash_table_insert(feature_node* &in_node)
{
    return hash_table_insert(hash_node_to_key(in_node), in_node);
}
bool Sokoban_features::hash_table_insert(unsigned long in_hash_value, feature_node* &in_node)
// inserts with a time constant of log(n) where n = hash_table_size
// if the element exists the in_node is changed to the existing element so it can be used for futher processing
// return true if element is inserted and false if it already exists
{
    hash_node tmp_hash_node;
    tmp_hash_node.hash_value = in_hash_value;
    tmp_hash_node.ref_node = in_node;

    if (hash_table.size() > 0) { // test if has_table is empty
        int const start_hash_table = 0; // access this element as hash_table.begin()
        int start_itr = start_hash_table;
        int const end_hash_table = hash_table.size()-1; // access this element as hash_table.end()
        int end_itr = end_hash_table;

        if (start_itr == end_itr) { // test if has_table only consists of 1 element
            if (in_hash_value == hash_table.at(start_itr).hash_value) {
                // element exists return ptr. NOTE
                in_node = hash_table.at(start_itr).ref_node;
                return false;
            } else if (in_hash_value > hash_table.at(start_itr).hash_value) {
                hash_table.push_back(tmp_hash_node);
                return true; // node inserted after 1st element
            } else {
                hash_table.insert(hash_table.begin()+start_itr,tmp_hash_node);
                return true; // node inserted before 1st element
            }
        }

        //cout << "[DEBUG INFO: hash_table_insert] entering while loop for " << in_hash_value << endl;
        int tmp_itr;
        while (true) {
            tmp_itr = (end_itr-start_itr)/2;
            //cout << "[DEBUG INFO: hash_table_insert] tmp_itr has value " << tmp_itr << ", start " << start_itr << ", end " << end_itr << endl; // debug info

            if (tmp_itr == 0) {
                if (in_hash_value == hash_table.at(start_itr).hash_value) {
                    // element exists return ptr. NOTE
                    in_node = hash_table.at(start_itr).ref_node;
                    return false;
                } else if (in_hash_value > hash_table.at(start_itr).hash_value) {
                    if (in_hash_value == hash_table.at(end_itr).hash_value) {
                        // element exists return ptr. NOTE
                        in_node = hash_table.at(end_itr).ref_node;
                        return false;
                    } else if (in_hash_value > hash_table.at(end_itr).hash_value) {
                        if (end_itr == end_hash_table) {
                            hash_table.push_back(tmp_hash_node);
                            return true; // node inserted at end of list
                        } else {
                            hash_table.insert(hash_table.begin() + end_itr + 1,tmp_hash_node);
                            return true; // node inserted after end_ptr
                        }
                    } else {
                        hash_table.insert(hash_table.begin() + end_itr,tmp_hash_node);
                        return true; // node inserted between ptr's
                    }
                } else {
                    hash_table.insert(hash_table.begin() + start_itr,tmp_hash_node); // insert at start itr space
                    return true; // node inserted before 1st element
                }
            } else {
                if (in_hash_value == hash_table.at(start_itr+tmp_itr).hash_value) {
                    // element exists return ptr.
                    in_node = hash_table.at(start_itr+tmp_itr).ref_node;
                    return false;
                } else if (in_hash_value > hash_table.at(start_itr+tmp_itr).hash_value) {
                    start_itr += tmp_itr;
                } else {
                    end_itr -= tmp_itr;
                }
            }
        }
    } else {
        hash_table.push_back(tmp_hash_node);
        return true; // first element inserted
    }
    return true;
}

bool Sokoban_features::hash_table_exist(feature_node* &in_node)
{
    //cout << "[DEBUG: hashing] Hash key " << hash_node_to_key(in_node) << endl;
    return hash_table_exist(hash_node_to_key(in_node), in_node);
}
bool Sokoban_features::hash_table_exist(unsigned long in_hash_value, feature_node* &in_node)
{
    if (hash_table.size() > 0) { // test if has_table is empty
        int const start_hash_table = 0; // access this element as hash_table.begin()
        int start_itr = start_hash_table;
        int const end_hash_table = hash_table.size()-1; // access this element as hash_table.end()
        int end_itr = end_hash_table;

        int tmp_itr;
        while (true) {
            tmp_itr = (end_itr-start_itr)/2;
            //cout << "[DEBUG INFO: hash_table_insert] tmp_itr has value " << tmp_itr << ", start " << start_itr << ", end " << end_itr << endl; // debug info

            if (tmp_itr == 0) {
                if (in_hash_value == hash_table.at(start_itr).hash_value) {
                    in_node = hash_table.at(start_itr).ref_node;
                    return true;
                } else if (in_hash_value == hash_table.at(end_itr).hash_value) {
                    in_node = hash_table.at(end_itr).ref_node;
                    return true;
                } else {
                    return false;
                }
            } else {
                if (in_hash_value == hash_table.at(start_itr+tmp_itr).hash_value) {
                    in_node = hash_table.at(start_itr+tmp_itr).ref_node;
                    return true;
                } else if (in_hash_value > hash_table.at(start_itr+tmp_itr).hash_value) {
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

bool Sokoban_features::hash_table_delete(feature_node* &in_node)
{
    return hash_table_delete(hash_node_to_key(in_node), in_node);
}
bool Sokoban_features::hash_table_delete(unsigned long in_hash_value, feature_node* &in_node)
{
    if (hash_table.size() > 0) { // test if has_table is empty
        int const start_hash_table = 0; // access this element as hash_table.begin()
        int start_itr = start_hash_table;
        int const end_hash_table = hash_table.size()-1; // access this element as hash_table.end()
        int end_itr = end_hash_table;

        int tmp_itr;
        while (true) {
            tmp_itr = (end_itr-start_itr)/2;
            //cout << "[DEBUG INFO: hash_table_insert] tmp_itr has value " << tmp_itr << ", start " << start_itr << ", end " << end_itr << endl; // debug info

            if (tmp_itr == 0) {
                if (in_hash_value == hash_table.at(start_itr).hash_value) {
                    hash_table.erase(hash_table.begin() + start_itr);
                    return true;
                } else if (in_hash_value == hash_table.at(end_itr).hash_value) {
                    hash_table.erase(hash_table.begin() + end_itr);
                    return true;
                } else {
                    return false;
                }
            } else {
                if (in_hash_value == hash_table.at(start_itr+tmp_itr).hash_value) {
                    hash_table.erase(hash_table.begin() + start_itr+tmp_itr);
                    return true;
                } else if (in_hash_value > hash_table.at(start_itr+tmp_itr).hash_value) {
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
