function an = angle_between_vectors(a, b)

an = atan2(norm(cross(a,b)), dot(a,b));