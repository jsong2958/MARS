def wall_hugger(maze):
    path = []
    grid = maze.maze


    wallMap = {
        (0,1): "right",
        (1,0): "bot",
        (0,-1): "left",
        (-1,0): "top"
    }

    leftMap = {
        (0,1): "top",
        (1,0): "right",
        (0,-1): "bot",
        (-1,0): "left"
    }

    curPos = [0,0]
    forward = [0,1]
    path.append(curPos.copy())
    

    while (tuple(curPos) != maze.end):

        r = curPos[0]
        c = curPos[1]

        if (not getattr(grid[r][c], leftMap[tuple(forward)])):
            temp = forward[0]
            forward[0] = -1 * forward[1]
            forward[1] = temp
            

        while (getattr(grid[r][c], wallMap[tuple(forward)])):
            temp = forward[0]
            forward[0] = forward[1]
            forward[1] = -1 * temp

        print("Facing towards:", wallMap[tuple(forward)])


        curPos[0] += forward[0]
        curPos[1] += forward[1]

        copy = curPos.copy()
        print(copy)
        path.append(copy)


    return path


def bfs(maze):
    path = []
    directions = [[1,0],[0,1],[-1,0],[0,-1]]

    visited = []
    q = []
    q.append([0,0])
    visited[0].insert(0, False)

    while (q):
        size = len(q)

        for i in range(size):
            cur = q.pop()
            r = cur[0]
            c = cur[1]
            for dir in directions:
                row = r + dir[0]
                col = c + dir[1]


