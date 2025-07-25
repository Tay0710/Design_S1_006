def generate_left_right_wall_lines(filename="left_right_lines.txt", room_x=1.0, spacing=0.05):
    y_start = 0.1
    y_end = 0.9
    x = 0.5  # drone in center of room
    z = 0.5
    yaw = 90.0  # facing forward along +y
    fixed_max = 1000  # dummy mm value for other sensors

    num_points = int((y_end - y_start) / spacing) + 1
    lines = []

    for i in range(num_points):
        y = y_start + i * spacing
        timestamp = f"00:00:{i:02}:000"

        # Distances to left/right walls
        left_dist = int(x * 1000)                  # x=0.0
        right_dist = int((room_x - x) * 1000)      # x=1.0

        # LEFT wall point (at x = 0.0)
        line_left = f"({timestamp}, 0.00, {y:.2f}, {z:.2f}, {yaw:.2f}, {fixed_max}, {fixed_max}, 0, {right_dist}, {fixed_max}, {fixed_max})"
        lines.append(line_left)

        # RIGHT wall point (at x = 1.0)
        line_right = f"({timestamp}, 1.00, {y:.2f}, {z:.2f}, {yaw:.2f}, {fixed_max}, {fixed_max}, {left_dist}, 0, {fixed_max}, {fixed_max})"
        lines.append(line_right)

    with open("../sensor_logs/" + filename, "w") as f:
        f.write("\n".join(lines))

    print(f"✅ Wrote {len(lines)} wall points on left and right walls at z=0.5 while drone moves along y")

if __name__ == '__main__':
    generate_left_right_wall_lines()
