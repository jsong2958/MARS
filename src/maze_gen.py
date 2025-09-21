import copy
import random
import time
from operator import index

import pygame
from pygame.locals import *
from sympy import ceiling
from sympy.codegen.fnodes import merge
from algorithms import wall_hugger

class TreeNode:
    def __init__(self, data, parent=None, children=None):
        self.data = data
        self.parent = parent
        self.children = []
        if children:
            self.children = children
    def add_child(self, child):
        self.children.append(child)

def move(pos, dx=0, dy=0):
    return pos[0] + dx, pos[1] + dy

def get_common_node(node_fam_1, node_fam_2):
    for node_1 in node_fam_1:
        for node_2 in node_fam_2:
            if node_1.data == node_2.data:
                return node_1.data


class Tile:
    def __init__(self, top, left, bot, right, role):
        self.top = top
        self.left = left
        self.bot = bot
        self.right = right
        self.role = role

class Maze:
    def __init__(self, row, col, start, end, num_sols):
        self.row = row
        self.col = col
        self.start = start
        self.end = end
        self.num_sols = num_sols

        # initialize maze grid with walls
        self.maze = []
        coord_list = []
        for i in range(self.row):
            temp_row = []
            for j in range(self.col):
                temp_row.append(Tile(True, True, True, True, "normal"))
                coord_list.append((j,i))
            self.maze.append(temp_row)

        # initialize default starting pos for default config
        if self.start == "default":
            self.start = (0, 0)
        if self.end == "default":
            self.end = (self.row - 1, self.col - 1)

        # generate a random tile to start at and add to list of visited tiles
        visited_tiles = [self.end]
        coord_list.remove(self.end)

        solution_path = []
        branches = []
        is_sol = False

        print("Generating maze...")


        # loop until maze is complete
        complete = False
        while not complete:

            if not is_sol:

                loop_start_tile = self.start
            else:
                # generate a second random tile
                loop_start_tile = coord_list[random.randint(0, len(coord_list) - 1)]

            new_tile = loop_start_tile
            possible_moves = ["right", "down", "left", "up"]

            save_maze = copy.deepcopy(self.maze)
            curr_loop = []
            maze_history = []
            while not new_tile in visited_tiles:


                if new_tile in curr_loop:

                    while curr_loop[-1] != new_tile:
                        curr_loop.pop()
                        maze_history.pop()
                    curr_loop.pop()
                    self.maze = maze_history.pop()

                    if curr_loop:
                        prev_tile = curr_loop[-1]

                        if prev_tile[0] < new_tile[0]:
                            possible_moves = ["right", "down", "up"]
                        elif prev_tile[0] > new_tile[0]:
                            possible_moves = ["down", "left", "up"]
                        elif prev_tile[1] < new_tile[1]:
                            possible_moves = ["right", "down", "left"]
                        elif prev_tile[1] > new_tile[1]:
                            possible_moves = ["right", "left", "up"]
                    else:
                        possible_moves = ["right", "down", "left", "up"]



                curr_loop.append(new_tile)
                maze_history.append(copy.deepcopy(self.maze))

                # check for walls
                if new_tile[0] == 0:
                    possible_moves.remove("left")
                elif new_tile[0] == self.col-1:
                    possible_moves.remove("right")
                if new_tile[1] == 0:
                    possible_moves.remove("up")
                elif new_tile[1] == self.row-1:
                    possible_moves.remove("down")

                move_dir = possible_moves[random.randint(0,len(possible_moves)-1)]

                if move_dir == "right":
                    self.maze[new_tile[1]][new_tile[0]].right = False
                    new_tile = move(new_tile,dx=1)
                    self.maze[new_tile[1]][new_tile[0]].left = False
                    possible_moves = ["right", "down", "up"]
                elif move_dir == "down":
                    self.maze[new_tile[1]][new_tile[0]].bot = False
                    new_tile = move(new_tile,dy=1)
                    self.maze[new_tile[1]][new_tile[0]].top = False
                    possible_moves = ["right", "down", "left"]
                elif move_dir == "left":
                    self.maze[new_tile[1]][new_tile[0]].left = False
                    new_tile = move(new_tile,dx=-1)
                    self.maze[new_tile[1]][new_tile[0]].right = False
                    possible_moves = ["down", "left", "up"]
                else:
                    self.maze[new_tile[1]][new_tile[0]].top = False
                    new_tile = move(new_tile,dy=-1)
                    self.maze[new_tile[1]][new_tile[0]].bot = False
                    possible_moves = ["right", "left", "up"]



            if not is_sol:

                solution_path += curr_loop
                solution_path.append(self.end)

                is_sol = True

            visited_tiles += curr_loop
            for coord in curr_loop:
                coord_list.remove(coord)



            curr_loop.append(new_tile)
            # checks if this is the solution loop to skip adding to branches
            if len(visited_tiles) != len(curr_loop):
                branches.append(curr_loop)



            if len(visited_tiles) == self.row * self.col:
                complete = True


        merge_found = True
        while merge_found:
            merge_found = False
            for i, branch in enumerate(branches):
                for j in range(i+1, len(branches)):
                    if branch[0] == branches[j][-1]:
                        # merge first branch to end of second branch, remove first branch
                        branch.pop(0)
                        branches[j] += branch
                        branches.pop(i)
                        merge_found = True
                        break
                if merge_found:
                    break

        # map a pseudo tree of the maze branches
        solution_node = TreeNode(solution_path)

        maze_tree = [[solution_node]]
        layer_num = 0
        while branches:
            branches_to_remove = []
            branch_nodes = []
            for branch in branches:
                for prev_branch in maze_tree[layer_num]:
                    if branch[-1] in prev_branch.data:
                        branches_to_remove.append(branch)
                        new_node = TreeNode(branch, parent=prev_branch)
                        prev_branch.add_child(new_node)
                        branch_nodes.append(new_node)

                        break

            maze_tree.append(branch_nodes)
            for branch in branches_to_remove:
                branches.remove(branch)
            layer_num += 1


        edges = []
        for j, row in enumerate(self.maze):
            for i, tile in enumerate(row):
                if tile.right and i != self.col-1:
                    tile_1 = (i,j)
                    tile_2 = (i+1,j)
                    edges.append((tile_1,tile_2))

                if tile.bot and j != self.row-1:
                    tile_1 = (i,j)
                    tile_2 = (i,j+1)
                    edges.append((tile_1,tile_2))

        remove_candidates = []

        for tile_1, tile_2 in edges:
            tile_1_node = None
            tile_2_node = None

            for layer in maze_tree:
                for branch in layer:
                    if tile_1 in branch.data:
                        tile_1_node = branch
                    if tile_2 in branch.data:
                        tile_2_node = branch
                    if tile_1_node and tile_2_node:
                        break
                if tile_1_node and tile_2_node:
                    break


            tile_1_node_fam = [copy.copy(tile_1_node)]
            while tile_1_node.parent:
                tile_1_node = tile_1_node.parent
                tile_1_node_fam.append(copy.copy(tile_1_node))
            tile_2_node_fam = [copy.copy(tile_2_node)]
            while tile_2_node.parent:
                tile_2_node = tile_2_node.parent
                tile_2_node_fam.append(copy.copy(tile_2_node))




            common_node = get_common_node(tile_1_node_fam, tile_2_node_fam)

            if common_node != solution_path:
                continue

            distance_1 = 0
            distance_2 = 0

            if tile_1 in common_node:
                common_node_index_1 = common_node.index(tile_1)
            else:

                travel_tile = tile_1
                for node in tile_1_node_fam:
                    if node.data == common_node:
                        common_node_index_1 = common_node.index(travel_tile)
                    else:
                        distance_1 += len(node.data) - node.data.index(travel_tile) - 1
                        travel_tile = node.data[-1]
            if tile_2 in common_node:
                common_node_index_2 = common_node.index(tile_2)
            else:
                travel_tile = tile_2
                for node in tile_2_node_fam:
                    if node.data == common_node:
                        common_node_index_2 = common_node.index(travel_tile)
                    else:
                        distance_2 += len(node.data) - node.data.index(travel_tile) - 1
                        travel_tile = node.data[-1]

            distance = abs(common_node_index_1-common_node_index_2) + distance_1 + distance_2
            if distance > (self.row + self.col)/2:
                remove_candidates.append((tile_1, tile_2))


        remove_candidates.sort()

        if num_sols > len(remove_candidates):
            num_sols = len(remove_candidates)
        for i in range(num_sols):
            rand_index = ceiling(random.randrange(0, (len(remove_candidates)-1) ** 2) ** 0.5)
            edge = remove_candidates[rand_index]
            remove_candidates.remove(edge)

            tile_1_x = edge[0][0]
            tile_1_y = edge[0][1]
            tile_2_x = edge[1][0]
            tile_2_y = edge[1][1]

            if edge[0][1] == edge[1][1]:
                self.maze[tile_1_y][tile_1_x].right = False
                self.maze[tile_2_y][tile_2_x].left = False
            else:
                self.maze[tile_1_y][tile_1_x].bot = False
                self.maze[tile_2_y][tile_2_x].top = False
        print("Maze complete!")



    def disp_maze(self, sol=None, path=None):

        if path is None:
            path = []
        if sol is None:
            sol = []
        pygame.init()
        pygame.display.init()
        window = pygame.display.set_mode((600, 600))
        window.fill((0, 0, 0))
        white = (255, 255, 255)

        run = True
        # Creating a while loop
        while run:

            # Iterating over all the events received from
            # pygame.event.get()
            for event in pygame.event.get():

                # If the type of the event is quit
                # then setting the run variable to false
                if event.type == QUIT:
                    run = False

            startx = 100
            starty = 100
            side_length = 400/self.row

            y = starty
            for j, row in enumerate(self.maze):
                x = startx
                for i, tile in enumerate(row):
                    if tile.top:
                        pygame.draw.line(window, white, [x,y], [x+side_length,y], 2)
                    if tile.left and not (i==0 and j==0):
                        pygame.draw.line(window, white, [x,y], [x,y+side_length], 2)
                    if tile.bot:
                        pygame.draw.line(window, white, [x+side_length,y+side_length], [x,y+side_length], 2)
                    if tile.right and not (i==self.row-1 and j==self.col-1):
                        pygame.draw.line(window, white, [x+side_length,y+side_length], [x+side_length,y], 2)
                    x += side_length
                y += side_length

            sol_path = copy.copy(sol)
            lines_path = []
            if sol_path:
                if len(sol_path) != len(sol)+2:
                    sol_path.insert(0,(0,-1))
                    sol_path.append((sol[-1][0],sol[-1][1]+1))
                for tile in sol_path:
                    lines_path.append((100+tile[1]*side_length+side_length/2,100+tile[0]*side_length+side_length/2))
            if path:
                for tile in path:
                    lines_path.append((100+tile[1]*side_length+side_length/2,100+tile[0]*side_length+side_length/2))

            if lines_path:
                pygame.draw.lines(window, (255,0,0), False, lines_path, 4)


            # Draws the surface object to the screen.
            pygame.display.update()


if __name__ == "__main__":
    my_maze = Maze(10, 10, "default", "default", 0)
    path = wall_hugger(my_maze)
    my_maze.disp_maze(sol=path)


