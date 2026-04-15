import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline, splprep, splev
import sys
import math

# ----------------------------
# Smooth Path Planning
# ----------------------------

class SmoothPathPlanner:
    """Generate smooth curved paths using splines and Bezier curves"""
    
    def __init__(self, method='cubic_spline'):
        """
        Initialize path planner
        
        Args:
            method: 'cubic_spline', 'bezier', or 'bspline'
        """
        self.method = method
        self.path = None
        self.waypoints = None
    
    def plan_cubic_spline(self, start, goal, via_points=None, num_points=50):
        """
        Plan path using cubic spline interpolation
        
        Args:
            start: (x, y) starting position
            goal: (x, y) goal position
            via_points: Optional list of (x, y) intermediate points to pass through
            num_points: Number of points to generate along the path
        
        Returns:
            List of (x, y, heading) waypoints
        """
        # Create control points
        if via_points is None:
            # No via points - create smooth curve with intermediate control point
            # Place control point perpendicular to straight line for natural curve
            mid_x = (start[0] + goal[0]) / 2
            mid_y = (start[1] + goal[1]) / 2
            
            # Offset perpendicular to create curvature
            dx = goal[0] - start[0]
            dy = goal[1] - start[1]
            length = math.sqrt(dx**2 + dy**2)
            
            if length > 0:
                # Perpendicular offset (20% of distance)
                perp_x = -dy / length * length * 0.2
                perp_y = dx / length * length * 0.2
                
                control_points = [
                    start,
                    (mid_x + perp_x, mid_y + perp_y),
                    goal
                ]
            else:
                control_points = [start, goal]
        else:
            # Use provided via points
            control_points = [start] + via_points + [goal]
        
        # Extract x and y coordinates
        x_points = [p[0] for p in control_points]
        y_points = [p[1] for p in control_points]
        
        if len(control_points) < 2:
            print("Error: Need at least 2 points for spline")
            return None
        
        if len(control_points) == 2:
            # Straight line if only 2 points
            t = np.linspace(0, 1, num_points)
            x_path = x_points[0] + t * (x_points[1] - x_points[0])
            y_path = y_points[0] + t * (y_points[1] - y_points[0])
        else:
            # Create parameter for spline (distance along points)
            t_points = np.linspace(0, 1, len(control_points))
            
            # Create cubic splines
            cs_x = CubicSpline(t_points, x_points)
            cs_y = CubicSpline(t_points, y_points)
            
            # Generate smooth path
            t_path = np.linspace(0, 1, num_points)
            x_path = cs_x(t_path)
            y_path = cs_y(t_path)
        
        # Calculate headings (tangent to path)
        waypoints = []
        for i in range(len(x_path)):
            if i < len(x_path) - 1:
                dx = x_path[i+1] - x_path[i]
                dy = y_path[i+1] - y_path[i]
                heading = math.atan2(dy, dx)
            else:
                # Use previous heading for last point
                heading = waypoints[-1][2] if waypoints else 0
            
            waypoints.append((x_path[i], y_path[i], heading))
        
        self.path = (x_path, y_path)
        self.waypoints = waypoints
        self.control_points = control_points
        
        return waypoints
    
    def plan_bezier(self, start, goal, control_points=None, num_points=50):
        """
        Plan path using Bezier curve
        
        Args:
            start: (x, y) starting position
            goal: (x, y) goal position
            control_points: List of control points for Bezier curve
            num_points: Number of points to generate
        
        Returns:
            List of (x, y, heading) waypoints
        """
        if control_points is None:
            # Create default control points for smooth S-curve
            dx = goal[0] - start[0]
            dy = goal[1] - start[1]
            
            cp1 = (start[0] + dx * 0.33, start[1] + dy * 0.15)
            cp2 = (start[0] + dx * 0.67, start[1] + dy * 0.85)
            control_points = [cp1, cp2]
        
        # Construct Bezier curve points
        all_points = [start] + control_points + [goal]
        n = len(all_points) - 1
        
        t = np.linspace(0, 1, num_points)
        x_path = np.zeros(num_points)
        y_path = np.zeros(num_points)
        
        # Bezier formula
        for i, point in enumerate(all_points):
            # Bernstein polynomial
            bernstein = (math.comb(n, i) * 
                        np.power(1 - t, n - i) * 
                        np.power(t, i))
            x_path += point[0] * bernstein
            y_path += point[1] * bernstein
        
        # Calculate headings
        waypoints = []
        for i in range(len(x_path)):
            if i < len(x_path) - 1:
                dx = x_path[i+1] - x_path[i]
                dy = y_path[i+1] - y_path[i]
                heading = math.atan2(dy, dx)
            else:
                heading = waypoints[-1][2] if waypoints else 0
            
            waypoints.append((x_path[i], y_path[i], heading))
        
        self.path = (x_path, y_path)
        self.waypoints = waypoints
        self.control_points = all_points
        
        return waypoints
    
    def plan_bspline(self, start, goal, via_points=None, num_points=50, smoothing=0):
        """
        Plan path using B-spline (parametric spline)
        
        Args:
            start: (x, y) starting position
            goal: (x, y) goal position
            via_points: Optional list of intermediate points
            num_points: Number of points to generate
            smoothing: Smoothing factor (0 = interpolate, >0 = approximate)
        
        Returns:
            List of (x, y, heading) waypoints
        """
        # Create control points
        if via_points is None:
            control_points = [start, goal]
        else:
            control_points = [start] + via_points + [goal]
        
        # Extract coordinates
        x_points = np.array([p[0] for p in control_points])
        y_points = np.array([p[1] for p in control_points])
        
        # Create B-spline
        if len(control_points) >= 4:
            # Use k=3 (cubic) for smooth curves
            tck, u = splprep([x_points, y_points], s=smoothing, k=3)
        else:
            # Use k=2 (quadratic) for fewer points
            k = min(len(control_points) - 1, 2)
            tck, u = splprep([x_points, y_points], s=smoothing, k=k)
        
        # Evaluate B-spline
        u_new = np.linspace(0, 1, num_points)
        x_path, y_path = splev(u_new, tck)
        
        # Calculate headings
        waypoints = []
        for i in range(len(x_path)):
            if i < len(x_path) - 1:
                dx = x_path[i+1] - x_path[i]
                dy = y_path[i+1] - y_path[i]
                heading = math.atan2(dy, dx)
            else:
                heading = waypoints[-1][2] if waypoints else 0
            
            waypoints.append((x_path[i], y_path[i], heading))
        
        self.path = (x_path, y_path)
        self.waypoints = waypoints
        self.control_points = control_points
        
        return waypoints
    
    def plan(self, start, goal, via_points=None, num_points=50):
        """
        Plan path using the specified method
        
        Args:
            start: (x, y) starting position
            goal: (x, y) goal position
            via_points: Optional list of intermediate points
            num_points: Number of waypoints to generate
        
        Returns:
            List of (x, y, heading) waypoints
        """
        if self.method == 'cubic_spline':
            return self.plan_cubic_spline(start, goal, via_points, num_points)
        elif self.method == 'bezier':
            return self.plan_bezier(start, goal, via_points, num_points)
        elif self.method == 'bspline':
            return self.plan_bspline(start, goal, via_points, num_points)
        else:
            print(f"Unknown method: {self.method}")
            return None
    
    def visualize(self, title="Smooth Path Plan"):
        """Visualize the planned path"""
        if self.path is None:
            print("No path to visualize. Run plan() first.")
            return
        
        x_path, y_path = self.path
        
        plt.figure(figsize=(10, 10))
        
        # Plot the smooth path
        plt.plot(x_path, y_path, 'b-', linewidth=2, label='Planned Path')
        
        # Plot control points
        if hasattr(self, 'control_points'):
            cp_x = [p[0] for p in self.control_points]
            cp_y = [p[1] for p in self.control_points]
            plt.plot(cp_x, cp_y, 'ro--', markersize=10, linewidth=1, 
                    alpha=0.5, label='Control Points')
        
        # Mark start and goal
        plt.plot(x_path[0], y_path[0], 'go', markersize=15, label='Start')
        plt.plot(x_path[-1], y_path[-1], 'rs', markersize=15, label='Goal')
        
        # Plot waypoints (every Nth point for clarity)
        step = max(1, len(x_path) // 20)
        wp_x = x_path[::step]
        wp_y = y_path[::step]
        plt.plot(wp_x, wp_y, 'c.', markersize=8, alpha=0.6, label='Waypoints')
        
        # Draw heading arrows (every Nth point)
        for i in range(0, len(self.waypoints), step):
            x, y, heading = self.waypoints[i]
            arrow_len = 5  # cm
            dx = arrow_len * math.cos(heading)
            dy = arrow_len * math.sin(heading)
            plt.arrow(x, y, dx, dy, head_width=3, head_length=2, 
                     fc='orange', ec='orange', alpha=0.5)
        
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.xlabel('X (cm)', fontsize=12)
        plt.ylabel('Y (cm)', fontsize=12)
        plt.title(title, fontsize=14, fontweight='bold')
        plt.legend(loc='best')
        
        # Add info text
        info = (f"Method: {self.method}\n"
                f"Waypoints: {len(self.waypoints)}\n"
                f"Path length: {self.calculate_path_length():.1f} cm")
        plt.text(0.02, 0.98, info, transform=plt.gca().transAxes,
                verticalalignment='top', fontsize=10,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
    
    def calculate_path_length(self):
        """Calculate total path length"""
        if self.waypoints is None:
            return 0
        
        length = 0
        for i in range(len(self.waypoints) - 1):
            x1, y1, _ = self.waypoints[i]
            x2, y2, _ = self.waypoints[i+1]
            length += math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        return length
    
    def export_waypoints(self, filename=None):
        """Export waypoints to file or print to console"""
        if self.waypoints is None:
            print("No waypoints to export")
            return
        
        if filename:
            with open(filename, 'w') as f:
                f.write("# Smooth Path Waypoints\n")
                f.write("# Format: x y heading(radians)\n")
                for x, y, heading in self.waypoints:
                    f.write(f"{x:.2f} {y:.2f} {heading:.4f}\n")
            print(f"Waypoints exported to {filename}")
        else:
            print("\nGenerated Waypoints:")
            print("Format: x y heading(degrees)")
            print("-" * 40)
            for i, (x, y, heading) in enumerate(self.waypoints[::5], 1):  # Print every 5th
                print(f"WP{i:3d}: ({x:7.2f}, {y:7.2f}) heading: {math.degrees(heading):6.1f}°")
            print("-" * 40)
            print(f"Total waypoints: {len(self.waypoints)}")

# ----------------------------
# Command Line Interface
# ----------------------------
def main():
    if len(sys.argv) < 3:
        print("Smooth Path Planner - Generate curved paths using splines/Bezier")
        print("\nUsage:")
        print("  python smooth_path_planner.py <goal_x> <goal_y> [options]")
        print("\nOptions:")
        print("  --method <type>      Path method: cubic_spline, bezier, bspline (default: cubic_spline)")
        print("  --via <x1> <y1> ...  Intermediate waypoints to pass through")
        print("  --points <n>         Number of waypoints to generate (default: 50)")
        print("  --start <x> <y>      Starting position (default: 0 0)")
        print("  --visualize          Show path visualization")
        print("  --export <file>      Export waypoints to file")
        print("  --command-line       Print as controller.py command")
        print("\nExamples:")
        print("  python smooth_path_planner.py 100 100 --visualize")
        print("  python smooth_path_planner.py 150 50 --via 50 80 100 20 --visualize")
        print("  python smooth_path_planner.py 100 100 --method bezier --visualize")
        print("  python smooth_path_planner.py 200 0 --points 30 --command-line")
        sys.exit(1)
    
    # Parse arguments
    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    
    start = (0, 0)
    method = 'cubic_spline'
    num_points = 50
    via_points = None
    visualize = False
    export_file = None
    print_command = False
    
    i = 3
    while i < len(sys.argv):
        arg = sys.argv[i]
        
        if arg == '--method':
            method = sys.argv[i+1]
            i += 2
        elif arg == '--via':
            via_points = []
            i += 1
            while i < len(sys.argv) and not sys.argv[i].startswith('--'):
                x = float(sys.argv[i])
                y = float(sys.argv[i+1])
                via_points.append((x, y))
                i += 2
        elif arg == '--points':
            num_points = int(sys.argv[i+1])
            i += 2
        elif arg == '--start':
            start = (float(sys.argv[i+1]), float(sys.argv[i+2]))
            i += 3
        elif arg == '--visualize':
            visualize = True
            i += 1
        elif arg == '--export':
            export_file = sys.argv[i+1]
            i += 2
        elif arg == '--command-line':
            print_command = True
            i += 1
        else:
            print(f"Unknown argument: {arg}")
            sys.exit(1)
    
    # Create planner and generate path
    planner = SmoothPathPlanner(method=method)
    goal = (goal_x, goal_y)
    
    print(f"\n{'='*60}")
    print(f"Smooth Path Planner")
    print(f"{'='*60}")
    print(f"Method: {method}")
    print(f"Start: ({start[0]:.1f}, {start[1]:.1f})")
    print(f"Goal:  ({goal[0]:.1f}, {goal[1]:.1f})")
    if via_points:
        print(f"Via points: {len(via_points)}")
        for i, (x, y) in enumerate(via_points, 1):
            print(f"  {i}. ({x:.1f}, {y:.1f})")
    print(f"Waypoints to generate: {num_points}")
    print(f"{'='*60}\n")
    
    # Generate path
    waypoints = planner.plan(start, goal, via_points, num_points)
    
    if waypoints:
        print(f"✓ Path generated successfully!")
        print(f"  Total waypoints: {len(waypoints)}")
        print(f"  Path length: {planner.calculate_path_length():.1f} cm")
        
        # Print command for controller.py if requested
        if print_command:
            print(f"\n{'='*60}")
            print("Command for controller.py:")
            print(f"{'='*60}")
            # Use every 5th waypoint to avoid too many arguments
            step = max(1, len(waypoints) // 10)
            cmd_waypoints = waypoints[::step]
            cmd = "python3 controller.py"
            for x, y, heading in cmd_waypoints:
                cmd += f" {x:.1f} {y:.1f}"
            print(cmd)
            print(f"{'='*60}\n")
        
        # Export if requested
        if export_file:
            planner.export_waypoints(export_file)
        else:
            planner.export_waypoints()  # Print to console
        
        # Visualize if requested
        if visualize:
            planner.visualize(f"Smooth Path: {method}")
    else:
        print("✗ Path generation failed")

if __name__ == "__main__":
    main()
