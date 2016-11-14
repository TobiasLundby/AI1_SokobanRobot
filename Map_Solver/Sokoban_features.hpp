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
    bool hash_table_delete(unsigned long in_hash_value);

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
    tmp_vec1.push_back(1);
    tmp_vec1.push_back(99);
    tmp_vec1.push_back(142142);
    feature_node* tmp_node_1 = insert_child(root);

    vector< int > tmp_vec2;
    tmp_vec2.push_back(2);
    tmp_vec2.push_back(99);
    tmp_vec2.push_back(142142);
    feature_node* tmp_node_2 = insert_child(root);

    vector< int > tmp_vec3;
    tmp_vec3.push_back(3);
    tmp_vec3.push_back(99);
    tmp_vec3.push_back(142142);
    feature_node* tmp_node_3 = insert_child(root);

    vector< int > tmp_vec4;
    tmp_vec4.push_back(3);
    tmp_vec4.push_back(99);
    tmp_vec4.push_back(142112);
    feature_node* tmp_node_4 = insert_child(root);

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
    string str7;
    for (size_t i = 0; i < tmp_vec4.size(); i++) {
        str7 += to_string(tmp_vec4.at(i));
    }
    cout << "Combined string4 is " << str4 << " and hashed " << str_hash(str4) << endl;
    cout << "Combined string5 is " << str5 << " and hashed " << str_hash(str5) << endl;
    cout << "Combined string6 is " << str6 << " and hashed " << str_hash(str6) << endl;
    cout << "Combined string6 is " << str7 << " and hashed " << str_hash(str7) << endl;
    cout << "str4 and str5: " << (str_hash(str4)==str_hash(str5)) << endl;
    cout << "str5 and str6: " << (str_hash(str5)==str_hash(str6)) << endl;





    cout << "calc" << 1/2 << endl;


    cout << "str1 and str2: " << (str_hash(str1)==str_hash(str2)) << endl;
    cout << "str2 and str3: " << (str_hash(str2)==str_hash(str3)) << endl;
    cout << "str1 and str3: " << (str_hash(str1)==str_hash(str3)) << endl;


    hash_table_insert(str_hash(str4), tmp_node_1);
    hash_table_insert(str_hash(str5), tmp_node_2);
    hash_table_insert(str_hash(str6), tmp_node_3);
    //hash_table_insert(str_hash(str7), tmp_node_4);
    //hash_table_insert(str_hash(str5), tmp_node_2);

    //cout << hash_table_insert(str_hash(str4), tmp_node_1) << endl;
    //hash_table_insert(str_hash(str5), tmp_node_2);
    //hash_table_insert(str_hash(str6), tmp_node_3);
    for (size_t i = 0; i < hash_table.size(); i++) {
        cout << "element " << i << " contains " << hash_table.at(i).hash_value << " and " << hash_table.at(i).ref_node << endl;
    }


    // cout << endl << "[INFO] Delete element begin" << endl;
    // hash_table_delete(str_hash(str1));
    // for (size_t i = 0; i < hash_table.size(); i++) {
    //     cout << "element " << i << " contains " << hash_table.at(i).hash_value << " and " << hash_table.at(i).ref_node << endl;
    // }

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
    cout << endl;
    hash_node tmp_hash_node;
    tmp_hash_node.hash_value = in_hash_value;
    tmp_hash_node.ref_node = in_node;
    if (hash_table.size() > 0) {
        int tmp_table_id = hash_table.size()/2;
        int table_id_add = tmp_table_id ;
        while(true)
        {
            table_id_add /= 2;
            cout << tmp_table_id << " and " << table_id_add << endl;
            if (hash_table.at(tmp_table_id).hash_value == in_hash_value)
                return false;
            else if (hash_table.at(tmp_table_id).hash_value > in_hash_value)
                tmp_table_id -= table_id_add;
            else if (hash_table.at(tmp_table_id).hash_value < in_hash_value)
                tmp_table_id += table_id_add;
            if (table_id_add == 0) {

                if (tmp_table_id == 0) { // if second element insert
                    if (hash_table.at(tmp_table_id).hash_value == in_hash_value)
                        return false;
                    else if (hash_table.at(tmp_table_id).hash_value > in_hash_value) {
                        hash_table.insert(hash_table.begin(),tmp_hash_node);
                    } else {
                        hash_table.insert(hash_table.begin()+1,tmp_hash_node);
                    }
                } else if (tmp_table_id == hash_table.size()-1) { // if itr at last element
                    if (hash_table.at(tmp_table_id).hash_value == in_hash_value)
                        return false;
                    else if (hash_table.at(tmp_table_id).hash_value > in_hash_value)
                        hash_table.insert(hash_table.end()-1,tmp_hash_node);
                    else
                        hash_table.push_back(tmp_hash_node);
                }

                // if (tmp_table_id == 0) {
                //     if (hash_table.at(tmp_table_id).hash_value == in_hash_value)
                //         return false;
                // } else if (tmp_table_id == hash_table.size()-2) {
                //     if (hash_table.at(tmp_table_id-1).hash_value == in_hash_value
                //     or hash_table.at(tmp_table_id).hash_value == in_hash_value
                //     or hash_table.at(tmp_table_id+1).hash_value == in_hash_value)
                //         return false;
                // } else {
                //     if (hash_table.at(tmp_table_id-1).hash_value == in_hash_value
                //     or hash_table.at(tmp_table_id).hash_value == in_hash_value )
                //         return false;
                // }
                // if (hash_table.at(tmp_table_id).hash_value < in_hash_value) {
                //     hash_table.insert(hash_table.begin()+tmp_table_id+1,tmp_hash_node);
                // } else
                //     hash_table.insert(hash_table.begin()+tmp_table_id-1,tmp_hash_node);

                break;

            }
        }
    } else {
        hash_table.push_back(tmp_hash_node);
    }
    return true;
}

bool Sokoban_features::hash_table_delete(unsigned long in_hash_value)
{
    if (hash_table.size() > 0) {
        int tmp_table_id = hash_table.size()/2;
        int table_id_add = tmp_table_id;

        while(true)
        {
            table_id_add /= 2;
            cout << tmp_table_id << " and " << table_id_add << endl;
            if (hash_table.at(tmp_table_id).hash_value == in_hash_value)
            {
                // Delete entry from table
                cout << "Found element at " << tmp_table_id << endl;
                return true;
            }
            else if (hash_table.at(tmp_table_id).hash_value > in_hash_value)
            {
                cout << "subtracting" << endl;
                tmp_table_id -= table_id_add;
            }
            else if (hash_table.at(tmp_table_id).hash_value < in_hash_value)
            {
                cout << "adding" << endl;
                tmp_table_id += table_id_add;
            }
            if (table_id_add == 0) {
                if (tmp_table_id == 1) {
                    //Test
                }


                return false;
            }
        }

    } else {
        return false;
    }
}
