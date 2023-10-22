# This script contains functions for parsing .yaml configuration
# files. Functions have .yaml file relative location as input arguments,
# and return necessary data for corresponding tasks 

# parser_ch1() : For chapter 1, returns center locations of boxes 
# in the desired arrangement, where unit_box_1 corresponds
# to list index 0.

# parser_ch2() : For chapter 2, returns the first box location and
# a structural matrix for construction of desired object
# each entry in structural matrix corresponds to height in unit cubes
# at given location, entry [0][0] corresponds to first box location.
# For the standard cube task, x, y lengths and height notation is
# transformed into structural matrix form then returned. For more
# advanced shapes structural matrices will be added manually. 

import yaml

def parser_ch1(yaml_path: str):
    with open(yaml_path, "r") as stream:
        try:
            raw = yaml.safe_load(stream)
            box_locations = []
            for i in range(8):
                box_locations.append(raw['box_links'][i][::1])
            
            return(box_locations)
        except yaml.YAMLError as exc:
            return(exc)
        
def parser_ch2(yaml_path: str):
    with open(yaml_path, "r") as stream:
        try:
            raw = yaml.safe_load(stream)
            data = raw['chapter_2']

            if data['box_unit_x_columns'] is not None:
                cube_dim = data['box_unit_x_columns']
                structure_matrix = [[cube_dim for _ in range(cube_dim)] for _ in range(cube_dim)]
            else:
                structure_matrix = data['structure_matrix']

            print(structure_matrix)  # Print structure matrix for debugging

            box_locations = {'first_box': data['first_box_location'], 'structure_matrix': structure_matrix}

            return box_locations
        except yaml.YAMLError as exc:
            return exc
