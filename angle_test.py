angle_pairs = [(-247, 270)]

def normalize_angle(angle):
    angle %= 360

    if angle > 180:
        angle -= 360
    
    return angle


for navx, target in angle_pairs:
    print(f"Robot points to {navx}°, target should be {target}°")
    navx_n   = normalize_angle(navx)
    target_n = normalize_angle(target)
    print(f"Robot points to {navx_n}n°, target should be {target_n}n°")

    offset_a = target_n - navx_n
    
    if offset_a > 0:
        offset_b = 360 - offset_a
    else:
        offset_b = offset_a + 360

    print(f"Offset A: {offset_a}, offset B: {offset_b}")

    if abs(offset_a) > abs(offset_b):
        best_offset = offset_b
    else:
        best_offset = offset_a

    print(f"Right offset is {best_offset}°")