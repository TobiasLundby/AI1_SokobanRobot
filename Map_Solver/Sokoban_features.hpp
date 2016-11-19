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

	void print_node(feature_node* in_node);
	void solve();
    int  point_type(feature_node* in_node, point2D &inPoint);
    double calcualte_heuristic(feature_node* in_node);
    double calculate_euclidian_distance(point2D &inPoint1, point2D &inPoint2);
    int calculate_taxicab_distance(point2D &inPoint1, point2D &inPoint2);
    void update_nearest_goals(feature_node* in_node);
    bool hash_table_insert(unsigned long in_hash_value, feature_node* in_node);

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
            for (size_t i = 0; i < temp_node->boxes.size(); i++) {
                temp_node->box_goal_ref.push_back(1);
            }
            update_nearest_goals(temp_node);
			temp_node->worker_pos = map->get_worker();
			temp_node->worker_dir = NORTH;
            temp_node->cost_to_node = 0;
            root = temp_node; // Set root as temp node after relevant info is saved from Map object
        } else
            cout << "Trying to create new root in existing tree, please create a new feature tree and try again" << endl;
    } else {
        temp_node = new Sokoban_features::feature_node{parent_node,parent_node->depth+1};
		temp_node->boxes = parent_node->boxes;
        temp_node->box_goal_ref = parent_node->box_goal_ref;
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

		int new_nodes = 10;

		for (size_t i = 0; i < new_nodes; i++) {
			feature_node* tmp_node = insert_child(root);
			if (i == new_nodes -1) {
				tmp_node->worker_pos.x = 7;
				print_node(tmp_node);
			}
		}
	} else
		cout << "[INFO] Tree already exists; break solve" << endl;

    cout << "x: " << root->boxes.at(0).x << ", y: " << root->boxes.at(0).y << endl;
    point2D tmp_point;
    tmp_point.x = 2;
    tmp_point.y = 1;
    point2D tmp_point2;
    tmp_point2.x = 4;
    tmp_point2.y = 2;

    cout << endl;
    update_nearest_goals(root);
    for (size_t i = 0; i < root->boxes.size(); i++) {
        cout << "Box " << i << " has goal " << root->box_goal_ref.at(i) << " as nearest goal" << endl;
    }

    cout << endl << endl << "[INFO] Hashing test" << endl;
    hash<string> str_hash; // define a sting hashing func

    string str1 = "Test1";
    string str2 = "Test2";
    string str3 = "Test1";

    vector< int > tmp_vec1;
    tmp_vec1.push_back(5);
    tmp_vec1.push_back(99);
    tmp_vec1.push_back(142142);

    vector< int > tmp_vec2;
    tmp_vec2.push_back(2);
    tmp_vec2.push_back(99);
    tmp_vec2.push_back(142142);

    vector< int > tmp_vec3;
    tmp_vec3.push_back(3);
    tmp_vec3.push_back(39);
    tmp_vec3.push_back(142142);

    string str4;
    for (size_t i = 0; i < tmp_vec1.size(); i++) {
        str4 += to_string(tmp_vec1.at(i));
    }
    string str5;
    for (size_t i = 0; i < tmp_vec2.size(); i++) {
        str5 += to_string(tmp_vec2.at(i));
    }
    string str6;
    for (size_t i = 0; i < tmp_vec3.size(); i++) {
        str6 += to_string(tmp_vec3.at(i));
    }
    cout << "Combined string4 is " << str4 << " and hashed " << str_hash(str4) << endl;
    cout << "Combined string5 is " << str5 << " and hashed " << str_hash(str5) << endl;
    cout << "Combined string6 is " << str6 << " and hashed " << str_hash(str6) << endl;
    cout << "str4 and str5: " << (str_hash(str4)==str_hash(str5)) << endl;
    cout << "str5 and str6: " << (str_hash(str5)==str_hash(str6)) << endl;





    cout << "calc" << 1/2 << endl;


    cout << "str1 and str2: " << (str_hash(str1)==str_hash(str2)) << endl;
    cout << "str2 and str3: " << (str_hash(str2)==str_hash(str3)) << endl;
    cout << "str1 and str3: " << (str_hash(str1)==str_hash(str3)) << endl;

    hash_table_insert(str_hash(str6), root);
    hash_table_insert(str_hash(str4), root);
    hash_table_insert(str_hash(str5), root);
    cout << hash_table_insert(str_hash(str4), root) << endl;
    hash_table_insert(str_hash(str5), root);
    hash_table_insert(str_hash(str6), root);
    for (size_t i = 0; i < hash_table.size(); i++) {
        cout << "element " << i << " contains " << hash_table.at(i).hash_value << " and " << hash_table.at(i).ref_node << endl;
    }

    cout << endl << endl << "[INFO]" << endl;
    //cout << *hash_table.end() << endl;
    //int tmp_itr = hash_table.back();

}

int  Sokoban_features::point_type(feature_node* in_node, point2D &inPoint)
{
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

// Hash table methods
bool Sokoban_features::hash_table_insert(unsigned long in_hash_value, feature_node* in_node)
// inserts with a time constant of log(n) where n = hash_table_size
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
                return false;
            } else if (in_hash_value > hash_table.at(start_itr).hash_value) {
                hash_table.push_back(tmp_hash_node);
                return true; // node inserted after 1st element
            } else {
                hash_table.insert(hash_table.begin()+start_itr,tmp_hash_node);
                return true; // node inserted before 1st element
            }
        }

        int tmp_itr;
        while (true) {
            tmp_itr = (end_itr-start_itr)/2;

            if (tmp_itr == 0) {
                if (in_hash_value == hash_table.at(start_itr).hash_value) {
                    // element exists return ptr. NOTE
                    return false;
                } else if (in_hash_value > hash_table.at(start_itr).hash_value) {
                    if (in_hash_value == hash_table.at(end_itr).hash_value) {
                        // element exists return ptr. NOTE
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
                if (in_hash_value == hash_table.at(tmp_itr).hash_value) {
                    // element exists return ptr. NOTE
                    return false;
                } else if (in_hash_value > hash_table.at(tmp_itr).hash_value) {
                    start_itr = tmp_itr;
                } else {
                    end_itr = tmp_itr;
                }
            }
        }
    } else {
        hash_table.push_back(tmp_hash_node);
        return true; // first element inserted
    }
    return true;
}
