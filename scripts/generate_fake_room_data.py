import math
import random

def distance_to_wall(x, y, z, yaw_deg, direction, room_size):
    room_x, room_y, room_z = room_size
    yaw_rad = math.radians(yaw_deg)

    if direction == 'front':
        return min(5000, (room_x - x) / math.cos(yaw_rad)) if math.cos(yaw_rad) != 0 else 5000
    if direction == 'back':
        return min(5000, x / math.cos(yaw_rad)) if math.cos(yaw_rad) != 0 else 5000
    if direction == 'left':
        return min(5000, y / math.cos(yaw_rad + math.pi/2)) if math.cos(yaw_rad + math.pi/2) != 0 else 5000
    if direction == 'right':
        return min(5000, (room_y - y) / math.cos(yaw_rad - math.pi/2)) if math.cos(yaw_rad - math.pi/2) != 0 else 5000
    if direction == 'top':
        return room_z - z
    if direction == 'bottom':
        return z

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
