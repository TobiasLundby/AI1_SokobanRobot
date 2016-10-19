//
//  Map.hpp
//  AI1_Sokoban-solver_MM-TL
//
//  Created by Tobias Lundby on 26/09/16.
//  All files are licenced under the BSD 3-Clause (see LICENSE.md)
//

#pragma once

#define obstacle    1
#define freespace   2
#define box         3
#define goal        4
#define start       5
#define undefined   9

class Map
{
public:
	// Constructor, overload constructor, and destructor
	Map();
	Map(Map &source_map);
	//Map(const std::vector<int> & list);
	~Map();

	// Public variables
	Map *parent_map;

	// Public Methods
	bool load_map_from_file(string file_name);
	void print_map();
	void print_info();

	vector< vector<int> > get_map();

private:
	// Private variables
	vector< vector<int> > map_structure;

	int map_height      = 0;
	int map_width       = 0;
	int map_obstacles   = 0;
	bool empty_map 		= true; // true if empty; false if not empty

	// Private Methods
};

Map::Map()
{
	cout << "From class1: " << parent_map << endl;
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
				for (size_t i = 0; i < line.length(); i++) {
					//save, switch etc.
					if (line.at(i) == 'X') {
						row.push_back(obstacle);
					} else if (line.at(i) == '.') {
						row.push_back(freespace);
					} else if (line.at(i) == 'J') {
						row.push_back(box);
					} else if (line.at(i) == 'G') {
						row.push_back(goal);
					} else if (line.at(i) == 'M') {
						row.push_back(start);
					}
				}
				map_structure.push_back(row);
			}
		}
		map_file.close();
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
	if (!empty_map) {
		cout << endl << " *** Printing map ***" << endl << "  ";
	    for (size_t i = 0; i < map_structure.at(0).size(); i++)
	        cout << i+1 << " ";
	    cout << endl;
	    for (size_t row = 0; row < map_structure.size(); row++) {
	        if (map_structure.size() >= 9)
	            cout << row+1 << "  ";
	        else
	            cout << row+1 << " ";
	        for (size_t col = 0; col < map_structure.at(row).size(); col++) {
	            //cout << map_structure_loaded.at(row).at(col);
	            if (map_structure.at(row).at(col) == 1) {
	                cout << "⊞"; // obstacle
	            } else if (map_structure.at(row).at(col) == 2) {
	                cout << " "; // free space
	            } else if (map_structure.at(row).at(col) == 3) {
	                cout << "\033[1;31m×\033[0m"; // box
	            } else if (map_structure.at(row).at(col) == 4) {
	                cout << "\033[1;32m★\033[0m"; // goal
	            } else if (map_structure.at(row).at(col) == 5) {
	                cout << "\033[1;35mΔ\033[0m"; // start
	            }
	            if (col >= 9) {
	                cout << "  ";
	            } else {
	                cout << " ";
	            }
	        }
	        cout << endl;
	    }
	    cout << "⊞: Obstacle" << endl;
	    cout << " : Free space" << endl;
	    cout << "\033[1;31m×\033[0m: Box" << endl;
	    cout << "\033[1;32m★\033[0m: Goal" << endl;
	    cout << "\033[1;35mΔ\033[0m: Start / robot" << endl;
	    cout << " *** Printing map done ***" << endl << endl;
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

vector< vector<int> > Map::get_map()
{
	return map_structure;
}
