import math
import random

def distance_to_wall(x, y, z, yaw_deg, direction, room_size):
    room_x, room_y, room_z = room_size
    yaw_rad = math.radians(yaw_deg)

    # Clamp range to 5 meters (5000mm)
    max_range = 5.0

    if direction == 'front':
        dx = (room_x - x)
        return max(0, min(max_range, dx / math.cos(yaw_rad))) if math.cos(yaw_rad) > 0 else max_range
    if direction == 'back':
        dx = x
        return max(0, min(max_range, dx / -math.cos(yaw_rad))) if math.cos(yaw_rad) < 0 else max_range
    if direction == 'left':
        dy = y
        return max(0, min(max_range, dy / -math.sin(yaw_rad))) if math.sin(yaw_rad) < 0 else max_range
    if direction == 'right':
        dy = (room_y - y)
        return max(0, min(max_range, dy / math.sin(yaw_rad))) if math.sin(yaw_rad) > 0 else max_range
    if direction == 'top':
        return max(0, min(max_range, room_z - z))
    if direction == 'bottom':
        return max(0, min(max_range, z))

def generate_room_data(filename="test_data_1000_lines.txt", room_size=(5.0, 5.0, 3.0), spacing=0.3):
    points = []
    room_x, room_y, room_z = room_size
    z = 1.0  # Fixed flight height

    for i in range(int(room_x / spacing)):
        for j in range(int(room_y / spacing)):
            x = i * spacing
            y = j * spacing
            yaw = random.uniform(0, 360)
            timestamp = f"{i:02}:{j:02}:00:000"

            # Simulate distances in mm (up to wall)
            p1 = int(distance_to_wall(x, y, z, yaw, 'front', room_size) * 1000)
            p2 = int(distance_to_wall(x, y, z, yaw, 'back', room_size) * 1000)
            p3 = int(distance_to_wall(x, y, z, yaw, 'left', room_size) * 1000)
            p4 = int(distance_to_wall(x, y, z, yaw, 'right', room_size) * 1000)
            p5 = int(distance_to_wall(x, y, z, yaw, 'top', room_size) * 1000)
            p6 = int(distance_to_wall(x, y, z, yaw, 'bottom', room_size) * 1000)

            line = f"({timestamp}, {x:.2f}, {y:.2f}, {z:.2f}, {yaw:.2f}, {p1}, {p2}, {p3}, {p4}, {p5}, {p6})"
            points.append(line)

    with open(f"../sensor_logs/{filename}", "w") as f:
        f.write("\n".join(points))

    print(f"✅ Generated {len(points)} readings in {filename}")

if __name__ == '__main__':
    generate_room_data()
