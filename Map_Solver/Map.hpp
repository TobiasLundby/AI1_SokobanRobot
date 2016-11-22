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
	bool create_deadlock_free_map();
	void print_map();
	void print_map(point2D& in_worker_pos, vector< point2D > in_boxes_pos, bool print_descriptor);
	void print_map_simple(int map_type);
	void print_info();
	void print_goals();
	void print_boxes();
	int  map_point_type(int in_x, int in_y, int map_type);
	int  map_point_type(point2D &inPoint, int map_type);

	vector< vector<int> > get_map(int map_type);
	vector< point2D > get_goals();
	vector< point2D > get_boxes();
	point2D get_worker();
	int  get_width();
	int  get_height();

private:
	// Private variables
	vector< vector<int> >  map_worker; // outer vector holds rows and therefore the internal vector is the column, ex. map_worker.at(y).at(x)
	vector< vector<int> >  map_box;

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
	map_worker = source_map.get_map(worker);
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
				map_worker.push_back(row);
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

bool Map::create_deadlock_free_map()
{
	for (size_t i = 0; i < map_worker.size(); i++) { // copy map
		map_box.push_back(map_worker.at(i));
	}
	for (size_t y = 0; y < map_box.size(); y++) { // outer vector is y
		for (size_t x = 0; x < map_box.at(y).size(); x++) { // inner vector is x
			point2D tmp_point;
			tmp_point.x = x;
			tmp_point.y = y;
			if ( (x > 0 and x < map_box.at(y).size()-1) and (y > 0 and y < map_box.size()-1) ) {
				if ( ( (map_worker.at(y-1).at(x) == obstacle and map_worker.at(y).at(x+1) == obstacle)
					or (map_worker.at(y).at(x+1) == obstacle and map_worker.at(y+1).at(x) == obstacle)
					or (map_worker.at(y+1).at(x) == obstacle and map_worker.at(y).at(x-1) == obstacle)
					or (map_worker.at(y).at(x-1) == obstacle and map_worker.at(y-1).at(x) == obstacle) )
					and (map_point_type(tmp_point,worker) != goal) ) {

					map_box.at(y).at(x) = obstacle;
				}
			}
		}
	}
	// check rows for deadlozed zones
	for (size_t y = 0; y < map_box.size(); y++) {
		int last_freespace_x = 0;
		for (size_t x = 0; x < map_box.at(y).size(); x++) {
			bool create_deadlocked_zone = true;
			bool goal_in_zone = false;
			if ( (map_point_type(x,y, worker) == freespace) and (map_point_type(x-1,y, worker) == obstacle) and ( x > last_freespace_x) ) {
				for (size_t x_tmp = x; x_tmp < map_box.at(y).size(); x_tmp++) {
					if (map_point_type(x_tmp,y, worker) == goal)
						goal_in_zone = true;
					if (map_point_type(x_tmp,y, worker) == obstacle) {
						last_freespace_x = x_tmp-1;
						if (!goal_in_zone) {
							bool prev_row = false;
							bool next_row = false;
							for (size_t x_prev_y = x; x_prev_y <= last_freespace_x; x_prev_y++) {
								if (map_point_type(x_prev_y,y-1, worker) != obstacle)
									prev_row = true;
								if (map_point_type(x_prev_y,y+1, worker) != obstacle)
									next_row = true;
							}
							if (!prev_row or !next_row)
								for (size_t x_mark = x; x_mark < last_freespace_x; x_mark++)
									map_box.at(y).at(x_mark) = obstacle;
						}
						break;
					}
				}
			}
		}
	}
	// check cols for deadlozed zones
	for (size_t x = 0; x < map_box.at(0).size(); x++) {
	    int last_freespace_y = 0;

	    for (size_t y = 0; y < map_box.size(); y++) {
	        bool create_deadlocked_zone = true;
	        bool goal_in_zone = false;
	        if ( (map_point_type(x,y, worker) == freespace) and (map_point_type(x,y-1, worker) == obstacle) and ( y > last_freespace_y) ) {
	            for (size_t y_tmp = y; y_tmp < map_box.size(); y_tmp++) {
	                if (map_point_type(x,y_tmp, worker) == goal)
	                    goal_in_zone = true;
	                if (map_point_type(x,y_tmp, worker) == obstacle) {
	                    last_freespace_y = y_tmp-1;
	                    if (!goal_in_zone) {
	                        bool prev_row = false;
	                        bool next_row = false;
	                        for (size_t y_prev_x = y; y_prev_x <= last_freespace_y; y_prev_x++) {
	                            if (map_point_type(x-1,y_prev_x, worker) != obstacle)
	                                prev_row = true;
	                            if (map_point_type(x+1,y_prev_x, worker) != obstacle)
	                                next_row = true;
	                        }
	                        if (!prev_row or !next_row)
	                            for (size_t y_mark = y; y_mark < last_freespace_y; y_mark++)
	                                map_box.at(y_mark).at(x) = obstacle;
	                    }
	                    break;
	                }
	            }
	        }
	    }
	}
	return true;
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
	    for (size_t i = 0; i < map_worker.at(0).size(); i++)
	        cout << i << " ";
	    cout << endl;
	    for (size_t row = 0; row < map_worker.size(); row++) {
	        if (map_worker.size() >= 10)
	            cout << row << "  ";
	        else
	            cout << row << " ";
	        for (size_t col = 0; col < map_worker.at(row).size(); col++) {
	            //cout << map_worker_loaded.at(row).at(col);
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


	            if (map_worker.at(row).at(col) == obstacle)
	                cout << "⊞"; // obstacle
	            else if (map_worker.at(row).at(col) == freespace and !printed_obj)
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

void Map::print_map_simple(int map_type)
{
	vector< vector<int> >* map_ptr;
	if (map_type == worker) {
		 map_ptr = &map_worker;
		 cout << endl << "[INFO] Printing worker map" << endl;
	} else if (map_type == box) {
		map_ptr = &map_box;
		cout << endl << "[INFO] Printing box map" << endl;
	} else {
		cout << "Unkown map type specified" << endl;
		return;
	}

	if (!empty_map) {
		cout << "  ";
	    for (size_t i = 0; i < map_ptr->at(0).size(); i++)
	        cout << i << " ";
	    cout << endl;
	    for (size_t row = 0; row < map_ptr->size(); row++) {
	        if (map_ptr->size() >= 10)
	            cout << row << "  ";
	        else
	            cout << row << " ";
	        for (size_t col = 0; col < map_ptr->at(row).size(); col++) {
	            if (map_ptr->at(row).at(col) == obstacle)
	                cout << "⊞"; // obstacle
	            else if (map_ptr->at(row).at(col) == freespace)
	                cout << " "; // free space
	            if (col >= 10)
	                cout << "  ";
	            else
	                cout << " ";
	        }
	        cout << endl;
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

vector< vector<int> > Map::get_map(int map_type)
{
	if (map_type == worker) {
		 return map_worker;
	} else if (map_type == box) {
		return map_box;
	} else {
		cout << "Unkown map type specified, returning worker map" << endl;
		return map_worker;
	}
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

int Map::map_point_type(int in_x, int in_y, int map_type)
{
	point2D tmp_point;
	tmp_point.x = in_x;
	tmp_point.y = in_y;
	return map_point_type(tmp_point, map_type);
}
int Map::map_point_type(point2D &inPoint, int map_type)
{
	vector< vector<int> >* map_ptr;
	if (map_type == worker) {
		 map_ptr = &map_worker;
	} else if (map_type == box) {
		map_ptr = &map_box;
	} else {
		cout << "Unkown map type specified" << endl;
		return undefined;
	}
	if ( (inPoint.x >= 0 and inPoint.x < map_width) and (inPoint.y >= 0 and inPoint.y < map_height) ) {
		for (size_t i = 0; i < initial_pos_goals.size(); i++)
	        if (initial_pos_goals.at(i).x == inPoint.x and initial_pos_goals.at(i).y == inPoint.y)
	            return goal;
		return map_ptr->at(inPoint.y).at(inPoint.x);
	} else {
		return undefined;
	}
}

int  Map::get_width()
{
	return map_width;
}
int  Map::get_height()
{
	return map_height;
}
