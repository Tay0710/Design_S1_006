import time

from x1_pixel_to_angular_rate import main as pixel_to_angular_rate
from x2_height_from_ToF import main as height_from_ToF
from x3_interpolate_heights import main as interpolate_heights
from optical_flow_method.x4_xy_velocity_calculation import main as xy_velocity_calculation


def main():
    t0 = time.time()

    print("\n=== Stage 1: pixel → angular-rate ===")
    pixel_to_angular_rate()

    print("\n=== Stage 2: ToF → height ===")
    height_from_ToF()

    print("\n=== Stage 3: interpolate heights ===")
    interpolate_heights()
    
    print("\n=== Stage 4: angular-rate + height → v_x, v_y ===")
    xy_velocity_calculation()

    print(f"\n✅ Pipeline complete in {time.time() - t0:.2f}s")

if __name__ == "__main__":
    main()