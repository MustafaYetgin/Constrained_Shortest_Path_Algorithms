import tkinter as tk
from tkinter import messagebox, scrolledtext
import datetime
import matplotlib.pyplot as plt
import constrainedAstar
import constrainedDijkstra
import constrainedBestFirstSearch
backend = 'TkAgg'
# Set the backend for the current session
plt.switch_backend(backend)

def draw_grid(shortest_path, grid_width, grid_height, obstacles, launch_point, target, all_opened, on_circle):

    plt.ion()
    plt.figure(figsize=(grid_width, grid_height))
    for x in range(grid_width + 1):
        plt.axvline(x, color='k', linestyle='--')

    for y in range(grid_height + 1):
        plt.axhline(y, color='k', linestyle='--')

    for obstacle in obstacles:
        one_corner_x = int(obstacle[0])
        one_corner_y = int(obstacle[1])
        x_square = [one_corner_x, one_corner_x, one_corner_x+1, one_corner_x+1]
        y_square = [one_corner_y, one_corner_y+1, one_corner_y+1 , one_corner_y]
        plt.fill(x_square,y_square,color='blue', alpha=1)

    for point in shortest_path:
        plt.plot([point[0]], [point[1]], marker='o', markersize=10, color='pink')


    for point in on_circle:
        plt.plot([point.column_val], [point.row_val], marker='.', markersize=10, color='green')

    plt.plot([launch_point[0]], [launch_point[1]], marker='o', markersize=10, color='none',
             markeredgecolor='purple')

    plt.plot([target[0]], [target[1]], marker='o', markersize=10, color='r')


    for opened in all_opened:

         if(not(len(opened) == 0)):
             curr_parent = opened[0].parent.column_val, opened[0].parent.row_val

         for node in opened:
             plt.plot([curr_parent[0], node.column_val], [curr_parent[1], node.row_val], color='brown')
             plt.plot([node.column_val], [node.row_val], marker='.', markersize=10, color='green')
             #plt.pause(0.1)
    if not(len(shortest_path)==1):
        for i in range(len(shortest_path) - 1):
            x1, y1 = shortest_path[i]
            x2, y2 = shortest_path[i + 1]
            plt.plot([x1, x2], [y1, y2], color='k',linewidth = 4)
    else:
        messagebox.showinfo("There is no path to target!!!!!")


    plt.axis('scaled')
    plt.axis('off')
    plt.ioff()

class ShortestPathUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Shortest Path Visualization")

        # Frame for input fields
        self.input_frame = tk.Frame(self.root)
        self.input_frame.pack(padx=10, pady=10)

        # Row and Column input fields
        self.row_label = tk.Label(self.input_frame, text="Row Number:")
        self.row_label.grid(row=0, column=0, padx=10, pady=5)
        self.row_entry = tk.Entry(self.input_frame, width=15)
        self.row_entry.grid(row=0, column=1, padx=10, pady=5)

        self.column_label = tk.Label(self.input_frame, text="Column Number:")
        self.column_label.grid(row=0, column=2, padx=10, pady=5)
        self.column_entry = tk.Entry(self.input_frame, width=15)
        self.column_entry.grid(row=0, column=3, padx=10, pady=5)

        # Launch and Target input fields
        self.launch_label = tk.Label(self.input_frame, text="Launch Point (x, y):")
        self.launch_label.grid(row=1, column=0, padx=10, pady=5)
        self.launch_entry = tk.Entry(self.input_frame, width=15)
        self.launch_entry.grid(row=1, column=1, padx=10, pady=5)

        self.target_label = tk.Label(self.input_frame, text="Target Point (x, y):")
        self.target_label.grid(row=1, column=2, padx=10, pady=5)
        self.target_entry = tk.Entry(self.input_frame, width=15)
        self.target_entry.grid(row=1, column=3, padx=10, pady=5)

        # Obstacle input field
        self.obstacle_label = tk.Label(self.input_frame, text="Obstacles (x1, y1; x2, y2; ...):")
        self.obstacle_label.grid(row=2, column=0, padx=10, pady=5)
        self.obstacle_text = scrolledtext.ScrolledText(self.input_frame, width=25, height=5)
        self.obstacle_text.grid(row=2, column=1, columnspan=3, padx=10, pady=5)

        self.discretization_label = tk.Label(self.input_frame, text="Discretization Sensitivity")
        self.discretization_label.grid(row=3, column=0, padx=10, pady=5)
        self.discretization_entry = tk.Entry(self.input_frame, width=15)
        self.discretization_entry.grid(row=3, column=1, padx=10, pady=5)

        self.searching_label = tk.Label(self.input_frame, text="Searching Sensitivity")
        self.searching_label.grid(row=3, column=2, padx=10, pady=5)
        self.searching_entry = tk.Entry(self.input_frame, width=15)
        self.searching_entry.grid(row=3, column=3, padx=10, pady=5)


        # Algorithm selection
        self.algorithm_var = tk.StringVar()
        self.algorithm_var.set("dijkstra")  # Default algorithm
        self.algorithm_menu = tk.OptionMenu(
            self.input_frame, self.algorithm_var, "dijkstra", "best_first", "a_star", "genetic_algorithm"
        )
        self.algorithm_menu.grid(row=4, column=1, padx=10, pady=5)

        # Button for finding the shortest path
        self.find_button = tk.Button(self.root, text="Find Shortest Path", command=self.find_shortest_path)
        self.find_button.pack(pady=10)

    def find_shortest_path(self):
        # Retrieve user input
        launch_coords = self.launch_entry.get()
        target_coords = self.target_entry.get()
        obstacle_coords = self.obstacle_text.get("1.0", tk.END)
        algorithm = self.algorithm_var.get()


        # Parse input into lists of coordinates
        try:
            row_num = int(self.row_entry.get())
            column_num = int(self.column_entry.get())
            searching_sensitivity = float(self.searching_entry.get())
            discretization_sensitivity = int(self.discretization_entry.get())
            launch_coords = [int(coord.strip()) for coord in launch_coords.split(",")]
            target_coords = [int(coord.strip()) for coord in target_coords.split(",")]
            if obstacle_coords.strip():
                obstacle_coords = [
                    [int(coord.strip()) for coord in obstacle.split(",")] for obstacle in obstacle_coords.split(";")
                ]
            else:
                obstacle_coords = []
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid coordinates.")
            return

        obstacle_check = True
        for obstacle in obstacle_coords:
            if obstacle == target_coords or obstacle == launch_coords or (not (column_num > obstacle[1] >= 0) or not (row_num > obstacle[0] >= 0)):
                obstacle_check = False

        if row_num < 0 or column_num < 0 or not (row_num > launch_coords[0] >= 0) or not (column_num > launch_coords[1] >= 0) or not (row_num > target_coords[0] >= 0) or not (column_num > target_coords[1] >= 0) or not (obstacle_check):
            messagebox.showerror("Input Error", "Please enter valid coordinates.")
            return

        # Call the selected algorithm and measure time
        all_opened = []
        on_circle = []
        target_node = None
        start_time = datetime.datetime.now()
        if algorithm == "dijkstra":
           target_node, all_opened, on_circle,closed,noChild,yesChild, length = constrainedDijkstra.constrained_dijkstra(launch_coords,target_coords,obstacle_coords,row_num,column_num,discretization_sensitivity,searching_sensitivity)
        elif algorithm == "best_first":
            target_node, all_opened, on_circle,closed,noChild,yesChild, length = constrainedBestFirstSearch.constrained_best_first_search(launch_coords,target_coords,obstacle_coords,row_num,column_num,discretization_sensitivity,searching_sensitivity)
        elif algorithm == "a_star":
            target_node, all_opened, on_circle,closed,noChild,yesChild, length = constrainedAstar.constrained_a_star(launch_coords,target_coords,obstacle_coords,row_num,column_num,discretization_sensitivity,searching_sensitivity)
        elif algorithm == "genetic_algorithm":
            pass
        else:
            messagebox.showerror("Algorithm Error", "Invalid algorithm selected.")
            return

        elapsed_time = datetime.datetime.now() - start_time

        shortest_path = []

        curr_node = target_node

        while not (curr_node is None):

            shortest_path.append([curr_node.column_val, curr_node.row_val])

            curr_node = curr_node.parent

        if shortest_path:
            draw_grid(shortest_path, column_num, row_num, obstacle_coords, launch_coords, target_coords, all_opened, on_circle)
            messagebox.showinfo("Elapsed Time", f"Elapsed Time: {elapsed_time.total_seconds():.6f} ms \n "
                                                f"Length Of The Path: {length:.6f} unit")
            plt.show()
        else:
            messagebox.showinfo("Elapsed Time", f"Elapsed Time: {elapsed_time.total_seconds():.6f} ms \n "
                                                f"Length Of The Path: {length:.6f} unit")
            messagebox.showinfo("No Path", "No path found to the target.")

        # Display elapsed time


# Create and run the GUI
if __name__ == "__main__":
    root = tk.Tk()
    app = ShortestPathUI(root)
    root.mainloop()



