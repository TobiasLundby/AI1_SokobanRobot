//
//  Map.hpp
//  AI1_Sokoban-solver_MM-TL
//
//  Created by Tobias Lundby on 26/09/16.
//  All files are licenced under the BSD 3-Clause (see LICENSE.md)
//

#pragma once

// Library include
// - none yet

// Class include
// - none yet

// Defines
#define obstacle    1
#define freespace   2
#define box         3
#define goal        4
#define start       5
#define worker 		6
#define undefined   9

// Namespaces
using namespace std;

class Map
{
public:
	// Constructor, overload constructor, and destructor
	Map();
	Map(Map &source_map);
	~Map();

	// Public variables
	Map *parent_map;

	// Public Methods
	bool load_map_from_file(string file_name);
	void print_map();
	void print_map(point2D& in_worker_pos, vector< point2D > in_boxes_pos, bool print_descriptor);
	void print_info();
	void print_goals();
	void print_boxes();
	int  map_point_type(point2D &inPoint);

	vector< vector<int> > get_map();
	vector< point2D > get_goals();
	vector< point2D > get_boxes();
	point2D get_worker();
	int  get_width();
	int  get_height();

private:
	// Private variables
	vector< vector<int> > map_structure; // outer vector holds rows and therefore the internal vector is the column, ex. map_structure.at(y).at(x)

	vector< point2D > initial_pos_goals;
	vector< point2D > initial_pos_boxes;
	point2D initial_pos_worker; // x,y

	int map_height      = 0;
	int map_width       = 0;
	int map_obstacles   = 0;
	bool empty_map 		= true; // true if empty; false if not empty

	// Private Methods
};

Map::Map()
{

}

Map::Map(Map &source_map)
{
	map_structure = source_map.get_map();
	empty_map = false; // Map has been transferred
	//print_map();

	//source_map.print_map();
	parent_map = &source_map;
	//cout << "From class2: " << parent_map << endl;

}

Map::~Map()
{
	// Do cleanup
}

bool Map::load_map_from_file(string file_name)
{
	string line;
	ifstream map_file (file_name);
	if (map_file.is_open())
	{
		cout << "Loading: " << file_name << endl;
		for (size_t i = 1; getline (map_file,line); i++) {
			if (i == 1) { // Catch first line which contains map info; XX YY DD, XX=width, YY=height, DD=obstacles
				map_width = atoi(line.substr(0, 2).c_str()); // atoi: convert from string to int
				map_height = atoi(line.substr(4, 2).c_str());
				map_obstacles = atoi(line.substr(7, 2).c_str());
			} else {
				//cout << line << '\n';
				vector<int> row; // Create an empty row
				for (size_t j = 0; j < line.length(); j++) {
					//save, switch etc.
					if (line.at(j) == 'X') {
						row.push_back(obstacle);
						//cout << "Obstacle at " << j << ", " << i << endl;
					} else if (line.at(j) == '.') {
						row.push_back(freespace);
					} else if (line.at(j) == 'J') {
						//row.push_back(box);
						row.push_back(freespace);
						initial_pos_boxes.push_back( point2D() );
						initial_pos_boxes.at(initial_pos_boxes.size()-1).x = j;
						initial_pos_boxes.at(initial_pos_boxes.size()-1).y = i-2;
					} else if (line.at(j) == 'G') {
						//row.push_back(goal);
						row.push_back(freespace);
						initial_pos_goals.push_back( point2D() );
						initial_pos_goals.at(initial_pos_goals.size()-1).x = j;
						initial_pos_goals.at(initial_pos_goals.size()-1).y = i-2;
					} else if (line.at(j) == 'M') {
						//row.push_back(start);
						row.push_back(freespace);
						initial_pos_worker.x = j;
						initial_pos_worker.y = i-2;
					}
				}
				map_structure.push_back(row);
			}
		}
		map_file.close();
		if (initial_pos_goals.size() != initial_pos_boxes.size()) {
			cout << "The number og goals and boxes does not match!" << endl << endl;
			return false;
		}
		cout << "Loading successful!" << endl << endl;
		empty_map = false;
		return true;
	} else {
		cout << "Unable to open file!" << endl;
		return false;
	}
}

void Map::print_map()
{
	print_map(initial_pos_worker, initial_pos_boxes, true);
}

void Map::print_map(point2D& in_worker_pos, vector< point2D > in_boxes_pos, bool print_descriptor)
{
	if (!empty_map) {
		if (print_descriptor)
			cout << endl << " *** Printing map ***" << endl;
		cout << "  ";
	    for (size_t i = 0; i < map_structure.at(0).size(); i++)
	        cout << i << " ";
	    cout << endl;
	    for (size_t row = 0; row < map_structure.size(); row++) {
	        if (map_structure.size() >= 10)
	            cout << row << "  ";
	        else
	            cout << row << " ";
	        for (size_t col = 0; col < map_structure.at(row).size(); col++) {
	            //cout << map_structure_loaded.at(row).at(col);
				bool printed_obj = false;

				if (in_worker_pos.x == col and in_worker_pos.y == row) {
					cout << "\033[1;35mΔ\033[0m"; // start
					printed_obj = true;
				}

				bool same_goal_box = false;
				for (size_t h = 0; h < in_boxes_pos.size(); h++){
					if (in_boxes_pos.at(h).x == col and in_boxes_pos.at(h).y == row) {
						for (size_t j = 0; j < initial_pos_goals.size(); j++) {
							if (initial_pos_goals.at(j).x == col and initial_pos_goals.at(j).y == row) {
								same_goal_box = true;
							}
						}
					}
				}
				if (same_goal_box) {
					cout << "\033[1;32mX\033[0m"; // box on goal
					printed_obj = true;
				} else {
					for (size_t h = 0; h < in_boxes_pos.size(); h++){
						if (in_boxes_pos.at(h).x == col and in_boxes_pos.at(h).y == row) {
							cout << "\033[1;31m×\033[0m"; // box
							printed_obj = true;
						}
					}
					for (size_t h = 0; h < initial_pos_goals.size(); h++) {
						if (initial_pos_goals.at(h).x == col and initial_pos_goals.at(h).y == row and !printed_obj) {
							cout << "\033[1;32m★\033[0m"; // goal
							printed_obj = true;
						}
					}
				}


	            if (map_structure.at(row).at(col) == obstacle)
	                cout << "⊞"; // obstacle
	            else if (map_structure.at(row).at(col) == freespace and !printed_obj)
	                cout << " "; // free space
	            if (col >= 10)
	                cout << "  ";
	            else
	                cout << " ";
	        }
	        cout << endl;
	    }
		if (print_descriptor) {
			cout << "⊞: Obstacle" << endl;
			cout << " : Free space" << endl;
			cout << "\033[1;31m×\033[0m: Box" << endl;
			cout << "\033[1;32m★\033[0m: Goal" << endl;
			cout << "\033[1;35mΔ\033[0m: Start / robot" << endl;
			cout << " *** Printing map done ***" << endl << endl;
		}
	} else {
		cout << "Map is empty and cannot be printed!" << endl << endl;
	}
}

void Map::print_info()
{
	cout << endl << "*** Map info ***" << endl;
	cout << "Width: " << map_width << endl;
	cout << "Height: " << map_height << endl;
	cout << "Obstacles: " << map_obstacles << endl;
}

void Map::print_goals()
{
	if (initial_pos_goals.size() > 0) {
		cout << endl << "Printing goals (" << initial_pos_goals.size() << ")" << endl;
		for (size_t i = 0; i < initial_pos_goals.size(); i++)
			cout << "Goal " << i << ": x=" << initial_pos_goals.at(i).x << ", y=" << initial_pos_goals.at(i).y << endl;
	} else
		cout << endl << "There are no goals in the map" << endl;
}

void Map::print_boxes()
{
	if (initial_pos_boxes.size() > 0) {
		cout << endl << "Printing boxes (" << initial_pos_boxes.size() << ")" << endl;
		for (size_t i = 0; i < initial_pos_boxes.size(); i++)
			cout << "Box " << i << ": x=" << initial_pos_boxes.at(i).x << ", y=" << initial_pos_boxes.at(i).y << endl;
	} else
		cout << endl << "There are no boxes in the map" << endl;
}

vector< vector<int> > Map::get_map()
{
	return map_structure;
}

vector< point2D > Map::get_goals()
{
	return initial_pos_goals;
}

vector< point2D > Map::get_boxes()
{
	return initial_pos_boxes;
}

point2D Map::get_worker()
{
	return initial_pos_worker;
}

int Map::map_point_type(point2D &inPoint)
{
	for (size_t i = 0; i < initial_pos_goals.size(); i++)
        if (initial_pos_goals.at(i).x == inPoint.x and initial_pos_goals.at(i).y == inPoint.y)
            return goal;
	return map_structure.at(inPoint.y).at(inPoint.x);
}

int  Map::get_width()
{
	return map_width;
}
int  Map::get_height()
{
	return map_height;
}
