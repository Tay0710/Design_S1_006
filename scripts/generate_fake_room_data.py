import math
import os

def get_wall_point(x, y, z, yaw_deg, distance, direction):
    yaw = math.radians(yaw_deg)
    if direction == 'front':
        return (x + distance * math.cos(yaw), y + distance * math.sin(yaw), z)
    elif direction == 'back':
        return (x - distance * math.cos(yaw), y - distance * math.sin(yaw), z)
    elif direction == 'left':
        return (x - distance * math.sin(yaw), y + distance * math.cos(yaw), z)
    elif direction == 'right':
        return (x + distance * math.sin(yaw), y - distance * math.cos(yaw), z)
    elif direction == 'top':
        return (x, y, z + distance)
    elif direction == 'bottom':
        return (x, y, z - distance)

def generate_wall_points(
        filename="test_room_points.txt",
        room_size=(5.0, 5.0, 3.0),
        spacing=0.2,
        yaw_steps=8):

    room_x, room_y, room_z = room_size
    yaw_angles = [i * (360 / yaw_steps) for i in range(yaw_steps)]
    drone_z = 1.0  # constant height

    all_points = []

    for i in range(int(room_x / spacing)):
        for j in range(int(room_y / spacing)):
            x = i * spacing
            y = j * spacing
            for yaw in yaw_angles:
                # Compute distances to all six surfaces
                front = room_x - x if math.cos(math.radians(yaw)) > 0 else x
                back  = x if math.cos(math.radians(yaw)) > 0 else room_x - x
                left  = y if math.sin(math.radians(yaw)) > 0 else room_y - y
                right = room_y - y if math.sin(math.radians(yaw)) > 0 else y
                top = room_z - drone_z
                bottom = drone_z

                distances = {
                    'front': min(front, 5),
                    'back': min(back, 5),
                    'left': min(left, 5),
                    'right': min(right, 5),
                    'top': min(top, 5),
                    'bottom': min(bottom, 5)
                }

                for direction, d in distances.items():
                    wall_point = get_wall_point(x, y, drone_z, yaw, d)
                    all_points.append((wall_point, direction))

    os.makedirs("../sensor_logs", exist_ok=True)
    with open("../sensor_logs/" + filename, "w") as f:
        for (px, py, pz), dir in all_points:
            yaw = 0.0  # not needed in final plot
            p1 = p2 = p3 = p4 = p5 = p6 = 5000
            if dir == 'front':  p1 = 0
            if dir == 'back':   p2 = 0
            if dir == 'left':   p3 = 0
            if dir == 'right':  p4 = 0
            if dir == 'top':    p5 = 0
            if dir == 'bottom': p6 = 0
            line = f"(00:00:00:000, {px:.2f}, {py:.2f}, {pz:.2f}, {yaw:.2f}, {p1}, {p2}, {p3}, {p4}, {p5}, {p6})"
            f.write(line + "\n")

    print(f"✅ Wrote {len(all_points):,} wall points to {filename}")
