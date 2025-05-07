import heapq
import numpy as np
import matplotlib.pyplot as plt
import time
import math
from matplotlib.animation import FuncAnimation

class Node:
    """A node class for A* Pathfinding"""
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0  # Cost from start to current node
        self.h = 0  # Heuristic (estimated cost from current to end)
        self.f = 0  # Total cost: f = g + h

    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f

def create_maze_from_string(maze_str):
    """Convertendo a representação em string do labirinto para um array 2D"""
    rows = maze_str.strip().split('\n')
    maze = []
    weights = []
    start = None
    end = None
    
    for i, row in enumerate(rows):
        current_row = []
        weight_row = []
        cols = row.split()
        for j, col in enumerate(cols):
            if col == 'S':
                start = (i, j)
                current_row.append(0)  # Treat 'S' as a free space (0)
                weight_row.append(1)   # Default weight for start
            elif col == 'E':
                end = (i, j)
                current_row.append(0)  # Treat 'E' as a free space (0)
                weight_row.append(1)   # Default weight for end
            elif col.startswith('W'):  # Format: W{weight}, e.g., W3 means weight 3
                weight = int(col[1:]) if len(col) > 1 else 2
                current_row.append(0)  # It's a free space
                weight_row.append(weight)  # But with a specific weight
            else:
                current_row.append(int(col))
                # Set weight 1 for free cells, infinity for obstacles
                weight_row.append(1 if int(col) == 0 else float('inf'))
        maze.append(current_row)
        weights.append(weight_row)
    
    return np.array(maze), np.array(weights), start, end

def manhattan_distance(pos1, pos2):
    """Calculate the Manhattan distance between two points"""
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

def astar(maze, weights, start, end, allow_diagonal=True, visualization_callback=None, delay=0.1):
    """
    A* path finding algorithm with support for diagonal movement and terrain weights
    
    Parameters:
    maze (2D array): The maze where 0 represents free cells and 1 represents obstacles
    weights (2D array): Weight matrix representing cost to traverse each cell
    start (tuple): Starting position coordinates  (linha, coluna)
    end (tuple): End position coordinates  (linha, coluna)
    allow_diagonal (bool): Whether to allow diagonal movement
    visualization_callback (function): Callback function for visualization during exploration
    delay (float): Delay between steps for visualization
    
    Returns:
    list: Path from start to end as a list of coordinates, or None if no path exists
    dict: Exploration history for visualization
    """
    # Validate input
    if not start or not end:
        return None, {}
    
    # Create start and end nodes
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize open and closed lists
    open_list = []  # Priority queue for nodes to be evaluated
    open_dict = {}  # Dictionary for quick lookup
    closed_set = set()  # Set of already evaluated nodes
    
    # Add the start node to the open list
    heapq.heappush(open_list, start_node)
    open_dict[start] = start_node
    
    # Define movement directions (up, right, down, left)
    adjacent_squares = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    
    # Add diagonal movements if allowed
    if allow_diagonal:
        adjacent_squares.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])
    
    # Dictionary to keep track of exploration history for visualization
    exploration_history = {
        'open_nodes': [],
        'closed_nodes': [],
        'current_node': [],
        'path': []
    }
    
    # Loop until the open list is empty
    while open_list:
        # Get the node with the lowest f value from the open list
        current_node = heapq.heappop(open_list)
        del open_dict[current_node.position]
        
        # Add the current node to the closed set
        closed_set.add(current_node.position)
        
        # Update exploration history
        exploration_history['closed_nodes'].append(current_node.position)
        exploration_history['current_node'] = [current_node.position]
        
        # Visualize if callback is provided
        if visualization_callback:
            visualization_callback(
                maze, 
                start, 
                end, 
                list(open_dict.keys()), 
                list(closed_set), 
                current_node.position,
                None
            )
            time.sleep(delay)
        
        # If the current node is the end node, reconstruct the path and return it
        if current_node.position == end_node.position:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            path = path[::-1]  # Reverse path to get from start to end
            
            # Update exploration history with the final path
            exploration_history['path'] = path
            
            # Final visualization with the complete path
            if visualization_callback:
                visualization_callback(
                    maze, 
                    start, 
                    end, 
                    list(open_dict.keys()), 
                    list(closed_set), 
                    None,
                    path
                )
                
            return path, exploration_history
        
        # Check all adjacent squares
        for new_position in adjacent_squares:
            # Get node position
            node_position = (current_node.position[0] + new_position[0], 
                             current_node.position[1] + new_position[1])
            
            # Make sure the position is within the maze boundaries
            if (node_position[0] < 0 or node_position[0] >= len(maze) or 
                node_position[1] < 0 or node_position[1] >= len(maze[0])):
                continue
            
            # Make sure the position is traversable (not an obstacle)
            if maze[node_position[0]][node_position[1]] == 1:
                continue
            
            # Skip if the node is in the closed set
            if node_position in closed_set:
                continue
            
            # Calculate movement cost (√2 for diagonal, cell weight for straight)
            is_diagonal = new_position[0] != 0 and new_position[1] != 0
            move_cost = math.sqrt(2) if is_diagonal else 1
            
            # Multiply by terrain weight
            move_cost *= weights[node_position[0]][node_position[1]]
            
            # Create a new node
            new_node = Node(current_node, node_position)
            
            # Calculate the new node's g, h, and f values
            new_node.g = current_node.g + move_cost
            new_node.h = manhattan_distance(new_node.position, end_node.position)
            new_node.f = new_node.g + new_node.h
            
            # Check if the node is already in the open list and if the new path is better
            if node_position in open_dict and new_node.g >= open_dict[node_position].g:
                continue
            
            # Add the new node to the open list
            heapq.heappush(open_list, new_node)
            open_dict[node_position] = new_node
            
            # Update exploration history
            exploration_history['open_nodes'].append(node_position)
    
    # No path found
    if visualization_callback:
        visualization_callback(
            maze, 
            start, 
            end, 
            list(open_dict.keys()), 
            list(closed_set), 
            None,
            None
        )
        
    return None, exploration_history

def visualize_maze_and_path(maze, start, end, path=None, weights=None):
    """
    Visualize the maze and the path if one is found
    
    Parameters:
    maze (2D array): The maze where 0 represents free cells and 1 represents obstacles
    start (tuple): Starting position coordinates  (linha, coluna)
    end (tuple): End position coordinates  (linha, coluna)
    path (list): Path from start to end as a list of coordinates
    weights (2D array): Weight matrix representing cost to traverse each cell
    """
    # Create a copy of the maze for visualization
    maze_visual = np.copy(maze)
    
    # Mark start and end positions
    maze_visual[start] = 2  # Start position marked as 2
    maze_visual[end] = 3    # End position marked as 3
    
    # Mark path positions if a path exists
    if path:
        for pos in path[1:-1]:  # Skip start and end positions
            maze_visual[pos] = 4  # Path positions marked as 4
    
    # Create a colormap for visualization
    colors = ['white', 'black', 'green', 'red', 'blue']
    cmap = plt.matplotlib.colors.ListedColormap(colors)
    bounds = [-.5, .5, 1.5, 2.5, 3.5, 4.5]
    norm = plt.matplotlib.colors.BoundaryNorm(bounds, cmap.N)
    
    # Create the figure and axis for plotting
    fig, axes = plt.subplots(1, 2, figsize=(16, 8))
    
    # Plot the maze with path
    axes[0].imshow(maze_visual, cmap=cmap, norm=norm)
    axes[0].grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
    axes[0].set_xticks(np.arange(-.5, len(maze[0]), 1), [])
    axes[0].set_yticks(np.arange(-.5, len(maze), 1), [])
    axes[0].set_title('Maze with Path')
    
    # Plot the weights if provided
    if weights is not None:
        # Create a terrain heatmap for the weights
        # Skip obstacles (which have infinite weight)
        weights_visual = np.copy(weights)
        weights_visual[weights_visual == float('inf')] = np.nan
        
        # Create a colormap for weights
        terrain_cmap = plt.cm.YlOrRd
        im = axes[1].imshow(weights_visual, cmap=terrain_cmap)
        
        # Add colorbar
        cbar = fig.colorbar(im, ax=axes[1])
        cbar.set_label('Terrain Cost')
        
        # Mark start and end on the weight map
        axes[1].plot(start[1], start[0], 'go', markersize=10)  # Green for start
        axes[1].plot(end[1], end[0], 'ro', markersize=10)      # Red for end
        
        # Plot the path if available
        if path:
            path_y, path_x = zip(*path)
            axes[1].plot(path_x, path_y, 'b-', linewidth=2)
        
        axes[1].grid(which='major', axis='both', linestyle='-', color='k', linewidth=1)
        axes[1].set_title('Terrain Weights')
    
    plt.tight_layout()
    plt.suptitle('Maze Visualization', fontsize=16)
    plt.show()

def visualize_exploration_step(maze, start, end, open_nodes, closed_nodes, current_node=None, path=None):
    """
    Visualize a single step of the A* algorithm exploration process
    
    Parameters:
    maze (2D array): The maze where 0 represents free cells and 1 represents obstacles
    start (tuple): Starting position coordinates  (linha, coluna)
    end (tuple): End position coordinates  (linha, coluna)
    open_nodes (list): List of nodes in the open set
    closed_nodes (list): List of nodes in the closed set
    current_node (tuple): Current node being explored
    path (list): Complete path if the algorithm has finished
    """
    # Create a copy of the maze for visualization
    maze_visual = np.copy(maze)
    
    # Mark closed nodes
    for node in closed_nodes:
        if node != start and node != end:
            maze_visual[node] = 5  # Closed nodes (already evaluated)
    
    # Mark open nodes
    for node in open_nodes:
        if node != start and node != end:
            maze_visual[node] = 6  # Open nodes (to be evaluated)
    
    # Mark the current node being explored
    if current_node and current_node != start and current_node != end:
        maze_visual[current_node] = 7  # Current node
        
    # Mark the final path if available
    if path:
        for pos in path[1:-1]:  # Skip start and end
            if pos != start and pos != end:
                maze_visual[pos] = 4  # Path
    
    # Mark start and end positions (should be on top of everything else)
    maze_visual[start] = 2  # Start
    maze_visual[end] = 3    # End
    
    # Create a colormap for visualization
    colors = ['white', 'black', 'green', 'red', 'blue', 'lightgray', 'yellow', 'orange']
    cmap = plt.matplotlib.colors.ListedColormap(colors)
    bounds = [-.5, .5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 7.5]
    norm = plt.matplotlib.colors.BoundaryNorm(bounds, cmap.N)
    
    # Clear the current figure and create a new one
    plt.clf()
    plt.figure(1)
    plt.imshow(maze_visual, cmap=cmap, norm=norm)
    
    # Add grid lines
    plt.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
    plt.xticks(np.arange(-.5, len(maze[0]), 1), [])
    plt.yticks(np.arange(-.5, len(maze), 1), [])
    
    # Add a legend
    legend_elements = [
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='white', markersize=15, label='Free Cell'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='black', markersize=15, label='Obstacle'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='green', markersize=15, label='Start'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='red', markersize=15, label='End'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='blue', markersize=15, label='Path'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='lightgray', markersize=15, label='Closed Node'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='yellow', markersize=15, label='Open Node'),
        plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='orange', markersize=15, label='Current Node')
    ]
    plt.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, -0.05), 
               fancybox=True, shadow=True, ncol=4)
    
    # Set title based on algorithm state
    if path:
        plt.title('A* PathFinder - Path Found!')
    elif current_node:
        plt.title(f'A* PathFinder - Exploring From {current_node}')
    else:
        plt.title('A* PathFinder - No Path Found')
    
    # Draw and pause briefly to update the display
    plt.draw()
    plt.pause(0.001)  # This allows the GUI to update

def print_maze_with_path(maze, start, end, path=None, weights=None):
    """
    Print the maze with the path highlighted and weights if provided
    
    Parameters:
    maze (2D array): The maze where 0 represents free cells and 1 represents obstacles
    start (tuple): Starting position coordinates  (linha, coluna)
    end (tuple): End position coordinates  (linha, coluna)
    path (list): Path from start to end as a list of coordinates
    weights (2D array): Weight matrix representing cost to traverse each cell
    """
    # Create a copy of the maze for printing
    maze_print = []
    for i in range(len(maze)):
        row = []
        for j in range(len(maze[i])):
            if (i, j) == start:
                row.append('S')
            elif (i, j) == end:
                row.append('E')
            elif path and (i, j) in path and (i, j) != start and (i, j) != end:
                row.append('*')
            else:
                if weights is not None and maze[i][j] == 0 and weights[i][j] > 1:
                    # Show weighted cells with their weight
                    row.append(f'W{int(weights[i][j])}')
                else:
                    row.append(str(maze[i][j]))
        maze_print.append(row)
    
    # Print the maze
    for row in maze_print:
        print(' '.join(row))
    
    # Print path details if available
    if path:
        print("\nPath coordinates:")
        path_coords = []
        for i, pos in enumerate(path):
            if pos == start:
                path_coords.append(f"S({pos[0]}, {pos[1]})")
            elif pos == end:
                path_coords.append(f"E({pos[0]}, {pos[1]})")
            else:
                path_coords.append(f"({pos[0]}, {pos[1]})")
        print(" -> ".join(path_coords))
        
        # Calculate path cost if weights are provided
        if weights is not None:
            total_cost = 0
            for i in range(1, len(path)):
                # Check if the move is diagonal
                prev_pos, curr_pos = path[i-1], path[i]
                is_diagonal = (prev_pos[0] != curr_pos[0]) and (prev_pos[1] != curr_pos[1])
                move_cost = math.sqrt(2) if is_diagonal else 1
                move_cost *= weights[curr_pos[0]][curr_pos[1]]
                total_cost += move_cost
            print(f"Total path cost: {total_cost:.2f}")



def main():
    """Função do Programa Principal"""
    print("Vizalização do A* Pathfinding ")
    print("--------------------------")
    
    print("\nOptions:")
    print("1. Rodar com labirinto de exemplo")
    print("2. Criar labirinto customizado")
    
    choice = input("Insra uma opção (1-2): ")
    
    if choice == '1':
        run_pathfinder()
    elif choice == '2':
        create_custom_maze()
    else:
        print("Opção Inválida. Saindo...")

if __name__ == "__main__":
    main()