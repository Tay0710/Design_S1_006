from format_data import read_file
import matplotlib.pyplot as plt

def plot_2d(timestamps, positions, distances):
    
    x, y, _ = zip(*positions)
    p_values = list(zip(*distances))
    sensor_labels = ['Front (p1)', 'Back (p2)', 'Left (p3)', 'Right (p4)', 'Top (p5)', 'Bottom (p6)']

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Plot of the positions
    ax1.plot(x, y, marker='o')
    ax1.set_title('2D Drone Path (X vs Y)')
    ax1.set_xlabel('X position')
    ax1.set_ylabel('Y position')
    ax1.grid(True)

    # Plot of the distances against time
    for i, p in enumerate(p_values):
        ax2.plot(timestamps, p, label=sensor_labels[i])
    ax2.set_title('Sensor Distances Over Time')
    ax2.set_xlabel('Timestamp')
    ax2.set_ylabel('Distance (mm)')
    ax2.tick_params(axis='x', rotation=45)
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    timestamps, positions, yaws, distances = read_file()
    plot_2d(timestamps, positions, distances)
