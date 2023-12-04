#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from random import randint
import json
import argparse
from typing import Union, List, Optional


class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

        # f = g + h
        self.f = 0
        self.g = 0
        self.h = 0

        self.neighbors: List[Node] = []
        self.previous = None
        self.is_obstacle = False

    def add_neighbors(self, grid, columns, rows):
        neighbor_x = self.x
        neighbor_y = self.y

        if neighbor_x < columns - 1:
            self.neighbors.append(grid[neighbor_x + 1][neighbor_y])
        if neighbor_x > 0:
            self.neighbors.append(grid[neighbor_x - 1][neighbor_y])
        if neighbor_y < rows - 1:
            self.neighbors.append(grid[neighbor_x][neighbor_y + 1])
        if neighbor_y > 0:
            self.neighbors.append(grid[neighbor_x][neighbor_y - 1])

        # diagonals
        """ if neighbor_x > 0 and neighbor_y > 0:
            self.neighbors.append(grid[neighbor_x-1][neighbor_y-1])
        if neighbor_x < columns -1 and neighbor_y > 0:
            self.neighbors.append(grid[neighbor_x+1][neighbor_y-1])
        if neighbor_x > 0 and neighbor_y <rows -1:
            self.neighbors.append(grid[neighbor_x-1][neighbor_y+1])
        if neighbor_x < columns -1 and neighbor_y < rows -1:
            self.neighbors.append(grid[neighbor_x+1][neighbor_y+1]) """


class AStar:
    def __init__(self, cols_n, rows_n, start_point, end_point, obstacle_ratio: Union[int, bool] = False, obstacle_list=False):
        self.cols_n = cols_n
        self.rows_n = rows_n
        self.start_point = start_point
        self.end_point = end_point
        self.obstacle_ratio = obstacle_ratio
        self.obstacle_list = obstacle_list

    @staticmethod
    def remove_node_from_open_set(open_set: List[Node], node: Node):
        for i in range(len(open_set)):
            if open_set[i] == node:
                open_set.pop(i)
                break

    @staticmethod
    def h_score(node: Node, end: Node):
        return abs(node.x - end.x) + abs(node.y - end.y)

    @staticmethod
    def create_grid(cols_n, rows_n, obstacle_ratio: Union[int, bool], obstacle_list: Union[list, bool]) -> List[List[Node]]:
        grid: List[List[Optional[Node]]] = []
        for i in range(cols_n):
            grid.append([])
            for j in range(rows_n):
                grid[-1].append(Node(i, j))
                if obstacle_ratio:
                    n = randint(0, 100)
                    if n < obstacle_ratio:
                        grid[i][j].is_obstacle = True

        if obstacle_list:
            for x, y in obstacle_list:
                grid[x][y].is_obstacle = True

        # Fill neighbors
        for i in range(cols_n):
            for j in range(rows_n):
                grid[i][j].add_neighbors(grid, cols_n, rows_n)

        return grid

    @staticmethod
    def iter(open_set: List[Node], closed_set: List[Node], end: Node):
        # Выбор наилучшей следующей опорной точки
        best_way_idx = 0
        for i in range(len(open_set)):
            if open_set[i].f < open_set[best_way_idx].f:
                best_way_idx = i

        current_node = open_set[best_way_idx]
        final_path = []

        # Проверка что мы достигли целевой точки
        if current_node == end:
            temp = current_node
            while temp.previous:
                final_path.append(temp.previous)
                temp = temp.previous

            return open_set, closed_set, current_node, final_path

        AStar.remove_node_from_open_set(open_set, current_node)
        closed_set.append(current_node)

        # Обход соседей опорной точки
        for neighbor in current_node.neighbors:
            if (neighbor in closed_set) or neighbor.is_obstacle:
                continue

            temp_g = current_node.g + 1
            is_in_open_set = False

            # Проверка что узел уже входит в open_set
            for it in open_set:
                if neighbor.x == it.x and neighbor.y == it.y:
                    # Если новая оценка лучше, то обновляем
                    if temp_g < it.g:
                        it.g = temp_g
                        it.h = AStar.h_score(it, end)
                        it.f = it.g + it.h
                        it.previous = current_node
                    is_in_open_set = True

            # Если не входит, то добавляем
            if not is_in_open_set:
                neighbor.g = temp_g
                neighbor.h = AStar.h_score(neighbor, end)
                neighbor.f = neighbor.g + neighbor.h
                neighbor.previous = current_node
                open_set.append(neighbor)

        return open_set, closed_set, current_node, final_path

    def main(self):
        grid = AStar.create_grid(self.cols_n, self.rows_n, self.obstacle_ratio, self.obstacle_list)

        open_set: List[Node] = []
        closed_set: List[Node] = []
        final_path: List[Node] = []
        open_set.append(grid[self.start_point[0]][self.start_point[1]])
        self.end_point = grid[self.end_point[0]][self.end_point[1]]

        while len(open_set) > 0:
            open_set, closed_set, current_node, final_path = AStar.iter(
                open_set, closed_set, self.end_point
            )
            if len(final_path) > 0:
                break

        return final_path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--cols", required=True)
    ap.add_argument("-r", "--rows", required=True)
    ap.add_argument("-s", "--start_x", required=True, help="x coor of start point")
    ap.add_argument("-q", "--start_y", required=True, help="y coor of start point")
    ap.add_argument("-e", "--end_x", required=True, help="x coor of end point")
    ap.add_argument("-t", "--end_y", required=True, help="y coor of end point")
    ap.add_argument("-o", "--obstacle_ratio", required=False, help="ratio of obstacle (black list)", default=20)
    ap.add_argument(
        "-l",
        "--obstacle_list",
        required=False,
        help="You can also create own obstacle. It should be list of list --> [[0,1],[3,2]] add in your obstacle.json file",
        default=False,
        type=str
    )

    args = vars(ap.parse_args())
    if args["obstacle_list"] == "True":
        json_file_path = "your_obstacle.json"
        with open(json_file_path, 'r') as f:
            contents = json.loads(f.read())
        data = json.loads(contents["data"])
        a_star = AStar(
            int(args["cols"]),
            int(args["rows"]),
            [int(args["start_x"]), int(args["start_y"])],
            [int(args["end_x"]),
             int(args["end_y"])],
            False,
            data
        )
    else:
        a_star = AStar(
            int(args["cols"]),
            int(args["rows"]),
            [int(args["start_x"]), int(args["start_y"])],
            [int(args["end_x"]), int(args["end_y"])],
            int(args["obstacle_ratio"]),
            False
        )

    output = a_star.main()
    if len(output) > 0:
        print("The way found!!!")
        for point in output:
            print(point.x, point.y)
    else:
        print("There is no legal way...You can decrease obstacle ration (default 20)")
