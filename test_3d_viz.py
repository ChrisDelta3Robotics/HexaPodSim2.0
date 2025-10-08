#!/usr/bin/env python3
"""
3D Visualization Test - Test matplotlib 3D capabilities
"""

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

def test_3d_matplotlib():
    """Test basic 3D matplotlib functionality"""
    print("Testing 3D matplotlib capabilities...")
    
    # Create a basic 3D plot
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    
    # Create a simple 3D wireframe cube
    # Define the vertices of a cube
    r = [-1, 1]
    X, Y = np.meshgrid(r, r)
    
    # Six faces of a cube
    ax.plot_surface(X, Y, np.ones_like(X), alpha=0.3, color='red')      # Top
    ax.plot_surface(X, Y, -np.ones_like(X), alpha=0.3, color='blue')    # Bottom
    ax.plot_surface(X, np.ones_like(X), Y, alpha=0.3, color='green')    # Front
    ax.plot_surface(X, -np.ones_like(X), Y, alpha=0.3, color='yellow')  # Back
    ax.plot_surface(np.ones_like(X), X, Y, alpha=0.3, color='orange')   # Right
    ax.plot_surface(-np.ones_like(X), X, Y, alpha=0.3, color='purple')  # Left
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Matplotlib Test - Cube')
    
    plt.show()
    print("3D matplotlib test completed")

def test_3d_in_tkinter():
    """Test 3D matplotlib embedded in tkinter"""
    print("Testing 3D matplotlib in tkinter...")
    
    root = tk.Tk()
    root.title("3D Matplotlib in Tkinter Test")
    root.geometry("800x600")
    
    # Create matplotlib figure
    fig = Figure(figsize=(8, 6), facecolor='black')
    ax = fig.add_subplot(111, projection='3d', facecolor='black')
    
    # Create a simple hexapod-like structure
    # Robot body (hexagon)
    angles = np.linspace(0, 2*np.pi, 7)
    body_radius = 0.1
    x_body = body_radius * np.cos(angles)
    y_body = body_radius * np.sin(angles)
    z_body = np.zeros_like(x_body)
    
    ax.plot(x_body, y_body, z_body, color='white', linewidth=3, label='Robot Body')
    
    # Robot legs (6 legs in star formation)
    leg_angles = np.array([0, 60, 120, 180, 240, 300])
    leg_colors = ['red', 'green', 'blue', 'yellow', 'orange', 'purple']
    
    for i, (angle, color) in enumerate(zip(leg_angles, leg_colors)):
        angle_rad = np.radians(angle)
        
        # Leg base on body
        base_x = 0.08 * np.cos(angle_rad)
        base_y = 0.08 * np.sin(angle_rad)
        base_z = 0
        
        # Leg extends outward and down
        foot_x = 0.2 * np.cos(angle_rad)
        foot_y = 0.2 * np.sin(angle_rad)
        foot_z = -0.15
        
        # Draw leg
        ax.plot([base_x, foot_x], [base_y, foot_y], [base_z, foot_z], 
               color=color, linewidth=2, label=f'Leg {i+1}')
        
        # Draw foot
        ax.scatter([foot_x], [foot_y], [foot_z], color=color, s=50)
    
    # Configure plot
    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(-0.2, 0.1)
    ax.set_xlabel('X (m)', color='white')
    ax.set_ylabel('Y (m)', color='white')
    ax.set_zlabel('Z (m)', color='white')
    ax.tick_params(colors='white')
    ax.set_title('Hexapod Robot 3D Visualization Test', color='white')
    
    # Ground grid
    grid_size = 0.3
    grid_spacing = 0.05
    x_grid = np.arange(-grid_size, grid_size + grid_spacing, grid_spacing)
    y_grid = np.arange(-grid_size, grid_size + grid_spacing, grid_spacing)
    
    for x in x_grid:
        ax.plot([x, x], [-grid_size, grid_size], [-0.2, -0.2], color='gray', alpha=0.3)
    for y in y_grid:
        ax.plot([-grid_size, grid_size], [y, y], [-0.2, -0.2], color='gray', alpha=0.3)
    
    # Embed in tkinter
    canvas = FigureCanvasTkAgg(fig, root)
    canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    # Add control buttons
    button_frame = tk.Frame(root)
    button_frame.pack(side=tk.BOTTOM, pady=10)
    
    def rotate_view():
        current_elev = ax.elev
        current_azim = ax.azim
        ax.view_init(elev=current_elev + 10, azim=current_azim + 10)
        canvas.draw()
    
    def reset_view():
        ax.view_init(elev=20, azim=45)
        canvas.draw()
    
    rotate_btn = tk.Button(button_frame, text="Rotate View", command=rotate_view)
    rotate_btn.pack(side=tk.LEFT, padx=5)
    
    reset_btn = tk.Button(button_frame, text="Reset View", command=reset_view)
    reset_btn.pack(side=tk.LEFT, padx=5)
    
    close_btn = tk.Button(button_frame, text="Close Test", command=root.quit)
    close_btn.pack(side=tk.LEFT, padx=5)
    
    print("3D tkinter window created. You should see:")
    print("- A hexapod robot with 6 colored legs")
    print("- Ground grid")
    print("- Rotate/Reset buttons")
    print("- 3D interactive view")
    
    root.mainloop()
    print("3D tkinter test completed")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Test 3D visualization')
    parser.add_argument('--basic', action='store_true', help='Test basic matplotlib 3D')
    parser.add_argument('--tkinter', action='store_true', help='Test 3D in tkinter')
    
    args = parser.parse_args()
    
    if args.basic:
        test_3d_matplotlib()
    elif args.tkinter:
        test_3d_in_tkinter()
    else:
        print("Testing 3D visualization capabilities...")
        print("Running tkinter embedded test...")
        test_3d_in_tkinter()