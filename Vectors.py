import sympy as sym

def find_intersection_LL(a1, b1, a2, b2):
    """
    Determines whether two parametric lines in 3D intersect, are skew, or are parallel.
    Parameters:
        a1, a2: Lists representing the position vectors (starting points of the lines)
        b1, b2: Lists representing the direction vectors of the lines
    """
    # Define parameters
    λ, μ = sym.symbols('λ μ')

    # Define parametric equations
    r1 = [a1[i] + λ * b1[i] for i in range(3)]
    r2 = [a2[i] + μ * b2[i] for i in range(3)]

    # Display parametric equations
    print("\nStep 1: Writing the parametric equations of the lines")
    print(f"Line 1: x = {a1[0]} + {b1[0]}λ,  y = {a1[1]} + {b1[1]}λ,  z = {a1[2]} + {b1[2]}λ")
    print(f"Line 2: x = {a2[0]} + {b2[0]}μ,  y = {a2[1]} + {b2[1]}μ,  z = {a2[2]} + {b2[2]}μ\n")

    # Step 2: Check if direction vectors are proportional (parallel test)
    print("Step 2: Checking if the direction vectors are proportional")

    ratios = []
    for i in range(3):
        if b2[i] != 0:  # Avoid division by zero
            ratios.append(b1[i] / b2[i])
        else:
            ratios.append(None)  # To handle division by zero properly

    print(f"Direction vector ratios: {ratios}")

    if all(ratio == ratios[0] for ratio in ratios if ratio is not None):  # If all non-None ratios are equal, lines are parallel
        print("Conclusion: The direction vectors are proportional, so the lines are parallel.")

        # Check if they are the same line (coincident)
        eq1 = sym.Eq(a1[0] + λ * b1[0], a2[0] + μ * b2[0])
        eq2 = sym.Eq(a1[1] + λ * b1[1], a2[1] + μ * b2[1])
        eq3 = sym.Eq(a1[2] + λ * b1[2], a2[2] + μ * b2[2])

        solution = sym.solve([eq1, eq2, eq3], (λ, μ))

        if solution:
            print("Step 3: The starting points satisfy the parametric equations, so the lines are the same (coincident).")
        else:
            print("Step 3: The starting points do not align, so the lines are parallel but distinct.")
        return
    else:
        print("Conclusion: The direction vectors are not proportional, so the lines are NOT parallel. Proceeding to check for intersection.\n")

    # Step 3: Set up and solve x and y equations
    print("Step 3: Solve for λ and μ using x and y components:")
    eq1 = sym.Eq(r1[0], r2[0])  # x-equation
    eq2 = sym.Eq(r1[1], r2[1])  # y-equation

    print("Equation 1 (x-component): ", sym.pretty(eq1))
    print("Equation 2 (y-component): ", sym.pretty(eq2))

    solution = sym.solve([eq1, eq2], (λ, μ))

    if solution:
        λ_value, μ_value = solution[λ], solution[μ]
        print(f"\nStep 4: Solving for λ and μ gives: λ = {λ_value}, μ = {μ_value}\n")

        # Step 5: Check z-component
        z1 = a1[2] + b1[2] * λ_value
        z2 = a2[2] + b2[2] * μ_value

        print("Step 5: Check if the z-components satisfy the same value")
        print(f"z1 = {a1[2]} + {b1[2]}({λ_value}) = {z1}")
        print(f"z2 = {a2[2]} + {b2[2]}({μ_value}) = {z2}")

        if z1 == z2:
            intersection = [r1[i].subs(λ, λ_value) for i in range(3)]
            print(f"\nStep 6: The z-equation holds, so the lines intersect at ({intersection[0]}, {intersection[1]}, {intersection[2]})")
        else:
            print("\nStep 6: The z-components do not match, so the lines are skew (do not intersect in 3D).")
    else:
        print("\nStep 4: No solution found for λ and μ, so the lines are skew (do not intersect).")


from sympy import symbols, Eq, solve,sqrt,Matrix

def find_intersection_PL(plane_normal, plane_constant, line_point, line_direction):
    """
    Find the intersection between a line and a plane using vector equations.

    Parameters:
    - plane_normal: Normal vector of the plane [a, b, c].
    - plane_constant: The constant d in the plane equation r · n = d.
    - line_point: A point on the line [x1, y1, z1].
    - line_direction: Direction vector of the line [u, v, w].

    Returns:
    - A string indicating the intersection point, no intersection, or if the line lies on the plane.
    """
    # Step 1: Unpack inputs
    a, b, c = plane_normal
    d = plane_constant
    x1, y1, z1 = line_point
    u, v, w = line_direction

    print(f"Step 1: Inputs")
    print(f"Plane normal vector: n = ({a}, {b}, {c})")
    print(f"Plane constant: d = {d}")
    print(f"Point on the line: a = ({x1}, {y1}, {z1})")
    print(f"Line direction vector: b = ({u}, {v}, {w})")
    print()

    # Step 2: Define the parameter λ (lambda)
    λ = symbols('λ')

    # Step 3: Write the vector equation of the line
    # r = a + λb
    r = (x1 + u * λ, y1 + v * λ, z1 + w * λ)

    print(f"Step 2: Vector equation of the line")
    print(f"r = a + λb")
    print(f"r = ({x1}, {y1}, {z1}) + λ({u}, {v}, {w})")
    print(f"r = ({r[0]}, {r[1]}, {r[2]})")
    print()

    # Step 4: Substitute the line equation into the plane equation
    # Plane equation: r · n = d
    plane_eq = a * r[0] + b * r[1] + c * r[2] - d

    print(f"Step 3: Substitute into the plane equation")
    print(f"Plane equation: r · n = d")
    print(f"({r[0]}, {r[1]}, {r[2]}) · ({a}, {b}, {c}) = {d}")
    print(f"Expanded equation: {a}*({r[0]}) + {b}*({r[1]}) + {c}*({r[2]}) = {d}")
    print(f"Simplified plane equation: {plane_eq} = 0")
    print()

    # Step 5: Solve for λ
    print(f"Step 4: Solve for λ")
    print(f"Solving the equation: {plane_eq} = 0")
    λ_solution = solve(plane_eq, λ)
    print(f"Solution for λ: {λ_solution}")
    print()

    # Step 6: Analyze the solution for λ
    if not λ_solution:
        print("Step 5: Analysis")
        print("No solution for λ. The line does not intersect the plane.")
        return "The line does not intersect the plane."
    elif isinstance(λ_solution, list) and len(λ_solution) == 0:
        print("Step 5: Analysis")
        print("No solution for λ. The line does not intersect the plane.")
        return "The line does not intersect the plane."
    elif isinstance(λ_solution, dict) and not λ_solution:
        print("Step 5: Analysis")
        print("No solution for λ. The line does not intersect the plane.")
        return "The line does not intersect the plane."
    else:
        # If λ_solution is a dictionary, extract the value
        if isinstance(λ_solution, dict):
            λ_value = λ_solution[λ]
        else:
            λ_value = λ_solution[0]

        print(f"Step 5: Analysis")
        print(f"Solution for λ: λ = {λ_value}")
        print()

        # Step 7: Substitute λ back into the line equation to find the intersection point
        intersection_point = (r[0].subs(λ, λ_value), r[1].subs(λ, λ_value), r[2].subs(λ, λ_value))

        print(f"Step 6: Find the intersection point")
        print(f"Substitute λ = {λ_value} into the line equation:")
        print(f"r = ({r[0].subs(λ, λ_value)}, {r[1].subs(λ, λ_value)}, {r[2].subs(λ, λ_value)})")
        print(f"Intersection point: {intersection_point}")
        print()

        # Step 8: Check if the line lies on the plane
        # If the plane equation is satisfied for all λ, the line lies on the plane
        if plane_eq.subs(λ, λ_value + 1) == plane_eq.subs(λ, λ_value):
            print("Step 7: Check if the line lies on the plane")
            print("The plane equation is satisfied for all λ. The line lies on the plane.")
            return "The line lies on the plane."
        else:
            print("Step 7: Check if the line lies on the plane")
            print("The plane equation is not satisfied for all λ. The line intersects the plane at a single point.")
            return f"The line intersects the plane at {intersection_point}"

# Example usage:
plane_normal = [1, 2, 3]  # Normal vector of the plane
plane_constant = 6         # d in the plane equation r · n = d
line_point = [1, 1, 1]     # A point on the line
line_direction = [1, 1, 1] # Direction vector of the line


def find_intersection_PP(plane1_normal, plane1_constant, plane2_normal, plane2_constant):
    """
    Find the intersection of two planes by finding two common points.

    Parameters:
    - plane1_normal: Normal vector of the first plane [a1, b1, c1].
    - plane1_constant: The constant d1 in the plane equation a1x + b1y + c1z = d1.
    - plane2_normal: Normal vector of the second plane [a2, b2, c2].
    - plane2_constant: The constant d2 in the plane equation a2x + b2y + c2z = d2.

    Returns:
    - A string indicating the intersection line, no intersection, or if the planes are the same.
    """
    # Step 1: Unpack inputs
    a1, b1, c1 = plane1_normal
    d1 = plane1_constant
    a2, b2, c2 = plane2_normal
    d2 = plane2_constant

    print(f"Step 1: Inputs")
    print(f"Plane 1: {a1}x + {b1}y + {c1}z = {d1}")
    print(f"Plane 2: {a2}x + {b2}y + {c2}z = {d2}")
    print()

    # Step 2: Define symbols
    x, y, z = symbols('x y z')

    # Step 3: Solve the system of equations to find two points
    equations = [
        a1 * x + b1 * y + c1 * z - d1,
        a2 * x + b2 * y + c2 * z - d2
    ]

    print(f"Step 2: Solve the system of equations to find two points")
    print(f"Equation 1: {a1}x + {b1}y + {c1}z = {d1}")
    print(f"Equation 2: {a2}x + {b2}y + {c2}z = {d2}")
    print()

    # Step 2.1: Fix z = 0 and solve for x and y
    print(f"Step 2.1: Fix z = 0 and solve for x and y")
    try:
        solution1 = solve([eq.subs(z, 0) for eq in equations], (x, y), dict=True)
        if not solution1:
            print("No solution found for z = 0. Trying another value.")
            # Step 2.2: Fix y = 0 and solve for x and z
            print(f"Step 2.2: Fix y = 0 and solve for x and z")
            solution1 = solve([eq.subs(y, 0) for eq in equations], (x, z), dict=True)
            if not solution1:
                print("No solution found. The planes do not intersect.")
                return "The planes do not intersect."
        point1 = {k: v for k, v in solution1[0].items()}
        if z in point1:
            point1[z] = 0  # Ensure z = 0 for the first point
        else:
            point1[z] = 0  # Add z = 0 to the point
        x1, y1, z1 = point1[x], point1[y], point1[z]
        print(f"Point 1: ({x1}, {y1}, {z1})")
    except Exception as e:
        print(f"Error solving for Point 1: {e}")
        return "The planes do not intersect or are parallel."

    # Step 2.3: Fix z = 1 and solve for x and y
    print(f"Step 2.3: Fix z = 1 and solve for x and y")
    try:
        solution2 = solve([eq.subs(z, 1) for eq in equations], (x, y), dict=True)
        if not solution2:
            print("No solution found for z = 1. Trying another value.")
            # Step 2.4: Fix y = 1 and solve for x and z
            print(f"Step 2.4: Fix y = 1 and solve for x and z")
            solution2 = solve([eq.subs(y, 1) for eq in equations], (x, z), dict=True)
            if not solution2:
                print("No solution found. The planes do not intersect.")
                return "The planes do not intersect."
        point2 = {k: v for k, v in solution2[0].items()}
        if z in point2:
            point2[z] = 1  # Ensure z = 1 for the second point
        else:
            point2[z] = 1  # Add z = 1 to the point
        x2, y2, z2 = point2[x], point2[y], point2[z]
        print(f"Point 2: ({x2}, {y2}, {z2})")
    except Exception as e:
        print(f"Error solving for Point 2: {e}")
        return "The planes do not intersect or are parallel."

    # Step 3: Parametric equation of the intersection line
    direction_vector = (x2 - x1, y2 - y1, z2 - z1)

    print(f"Step 3: Parametric equation of the intersection line")
    print(f"r = ({x1}, {y1}, {z1}) + λ({direction_vector[0]}, {direction_vector[1]}, {direction_vector[2]})")
    print()

    return f"The intersection line is r = ({x1}, {y1}, {z1}) + λ({direction_vector[0]}, {direction_vector[1]}, {direction_vector[2]})"


def shortest_distance_between_lines(line1_point, line1_direction, line2_point, line2_direction):
    """
    Find the shortest distance between two lines using the specified method.

    Parameters:
    - line1_point: A point on the first line [x1, y1, z1].
    - line1_direction: Direction vector of the first line [u1, v1, w1].
    - line2_point: A point on the second line [x2, y2, z2].
    - line2_direction: Direction vector of the second line [u2, v2, w2].

    Returns:
    - The shortest distance between the two lines.
    """
    # Step 1: Unpack inputs
    a1 = Matrix(line1_point)
    d1 = Matrix(line1_direction)
    a2 = Matrix(line2_point)
    d2 = Matrix(line2_direction)

    print(f"Step 1: Inputs")
    print(f"Line 1: r1 = ({a1[0]}, {a1[1]}, {a1[2]}) + λ({d1[0]}, {d1[1]}, {d1[2]})")
    print(f"Line 2: r2 = ({a2[0]}, {a2[1]}, {a2[2]}) + μ({d2[0]}, {d2[1]}, {d2[2]})")
    print()

    # Step 2: Check if the lines are parallel
    cross_d1_d2 = d1.cross(d2)
    if cross_d1_d2 == Matrix([0, 0, 0]):
        print("Step 2: The lines are parallel.")
        # Step 3: For parallel lines, use the method involving t = λ - μ
        t = symbols('t')
        a2_minus_a1 = a2 - a1
        print(f"Step 2.1: Subtract r2 - r1:")
        print(f"r2 - r1 = ({a2[0]} + μ{d2[0]}) - ({a1[0]} + λ{d1[0]}), ({a2[1]} + μ{d2[1]}) - ({a1[1]} + λ{d1[1]}), ({a2[2]} + μ{d2[2]}) - ({a1[2]} + λ{d1[2]}))")
        print(f"Simplify: ({a2_minus_a1[0]} + μ{d2[0]} - λ{d1[0]}, {a2_minus_a1[1]} + μ{d2[1]} - λ{d1[1]}, {a2_minus_a1[2]} + μ{d2[2]} - λ{d1[2]})")
        print()
        print("Explanation: Subtracting r2 - r1 gives the vector connecting a point on Line 2 to a point on Line 1.")
        print("This vector represents the displacement between the two lines at specific values of λ and μ.")
        print()

        # Since d1 and d2 are parallel, d2 = k d1 (for some scalar k)
        # Here, d2 = d1, so μ d2 - λ d1 = (μ - λ) d1
        # Let t = μ - λ
        eq = (a2_minus_a1 + t * d1).dot(d1)
        print(f"Step 2.2: Set (r2 - r1) · d1 = 0:")
        print(f"({a2_minus_a1[0]} + t{d1[0]}){d1[0]} + ({a2_minus_a1[1]} + t{d1[1]}){d1[1]} + ({a2_minus_a1[2]} + t{d1[2]}){d1[2]} = 0")
        print(f"Simplify: {eq} = 0")
        print()
        print("Explanation: To minimize the distance, the connecting vector (r2 - r1) must be perpendicular to the direction vector d1.")
        print("This ensures that the distance is the shortest possible.")
        print()

        t_value = solve(eq, t)[0]
        print(f"Step 2.3: Solve for t: t = {t_value}")
        print()
        print("Explanation: Solving the equation (r2 - r1) · d1 = 0 gives the value of t that minimizes the distance.")
        print()

        # Substitute t back into r2 - r1
        connecting_vector = a2_minus_a1 + t_value * d1
        print(f"Step 2.4: Substitute t back into r2 - r1:")
        print(f"r2 - r1 = ({a2_minus_a1[0]} + {t_value}{d1[0]}, {a2_minus_a1[1]} + {t_value}{d1[1]}, {a2_minus_a1[2]} + {t_value}{d1[2]}) = ({connecting_vector[0]}, {connecting_vector[1]}, {connecting_vector[2]})")
        print()
        print("Explanation: Substituting t back into r2 - r1 gives the connecting vector at the point of minimum distance.")
        print()

        distance = connecting_vector.norm()
        print(f"Step 2.5: Compute the norm of r2 - r1:")
        print(f"Distance = sqrt(({connecting_vector[0]})^2 + ({connecting_vector[1]})^2 + ({connecting_vector[2]})^2) = {distance}")
        print()
        print("Explanation: The norm of the connecting vector gives the shortest distance between the two lines.")
        print()

        print(f"Shortest distance between parallel lines: {distance}")
        return distance
    else:
        print("Step 2: The lines are skew.")
        # Step 3: For skew lines, solve for λ and μ
        λ, μ = symbols('λ μ')
        # Subtract r2 - r1
        r2_minus_r1 = (a2 + μ * d2) - (a1 + λ * d1)
        print(f"Step 2.1: Subtract r2 - r1:")
        print(f"r2 - r1 = ({a2[0]} + μ{d2[0]}) - ({a1[0]} + λ{d1[0]}), ({a2[1]} + μ{d2[1]}) - ({a1[1]} + λ{d1[1]}), ({a2[2]} + μ{d2[2]}) - ({a1[2]} + λ{d1[2]}))")
        print(f"Simplify: ({r2_minus_r1[0]}, {r2_minus_r1[1]}, {r2_minus_r1[2]})")
        print()
        print("Explanation: Subtracting r2 - r1 gives the vector connecting a point on Line 2 to a point on Line 1.")
        print("This vector represents the displacement between the two lines at specific values of λ and μ.")
        print()

        # Dot product with d1 and d2 to form two equations
        eq1 = r2_minus_r1.dot(d1)
        eq2 = r2_minus_r1.dot(d2)
        print(f"Step 2.2: Set (r2 - r1) · d1 = 0 and (r2 - r1) · d2 = 0:")
        print(f"(r2 - r1) · d1 = ({r2_minus_r1[0]}){d1[0]} + ({r2_minus_r1[1]}){d1[1]} + ({r2_minus_r1[2]}){d1[2]} = 0")
        print(f"Simplify: {eq1} = 0")
        print(f"(r2 - r1) · d2 = ({r2_minus_r1[0]}){d2[0]} + ({r2_minus_r1[1]}){d2[1]} + ({r2_minus_r1[2]}){d2[2]} = 0")
        print(f"Simplify: {eq2} = 0")
        print()
        print("Explanation: To minimize the distance, the connecting vector (r2 - r1) must be perpendicular to both direction vectors d1 and d2.")
        print("This ensures that the distance is the shortest possible.")
        print()

        # Solve the system of equations for λ and μ
        solution = solve((eq1, eq2), (λ, μ), dict=True)
        if not solution:
            print("No solution found. The lines do not intersect.")
            return "The lines do not intersect."
        λ_value = solution[0][λ]
        μ_value = solution[0][μ]
        print(f"Step 2.3: Solve for λ and μ: λ = {λ_value}, μ = {μ_value}")
        print()
        print("Explanation: Solving the system of equations gives the values of λ and μ that minimize the distance.")
        print()

        # Substitute λ and μ back into r2 - r1
        connecting_vector = r2_minus_r1.subs({λ: λ_value, μ: μ_value})
        print(f"Step 2.4: Substitute λ and μ back into r2 - r1:")
        print(f"r2 - r1 = ({connecting_vector[0]}, {connecting_vector[1]}, {connecting_vector[2]})")
        print()
        print("Explanation: Substituting λ and μ back into r2 - r1 gives the connecting vector at the point of minimum distance.")
        print()

        distance = connecting_vector.norm()
        print(f"Step 2.5: Compute the norm of r2 - r1:")
        print(f"Distance = sqrt(({connecting_vector[0]})^2 + ({connecting_vector[1]})^2 + ({connecting_vector[2]})^2) = {distance}")
        print()
        print("Explanation: The norm of the connecting vector gives the shortest distance between the two lines.")
        print()

        print(f"Shortest distance between skew lines: {distance}")
        return distance


def shortest_distance_line_point(line_point, line_direction, point):
    """
    Find the shortest distance between a line and a point using the described method.

    Parameters:
    - line_point: A point on the line [x0, y0, z0].
    - line_direction: Direction vector of the line [u, v, w].
    - point: The point [x_p, y_p, z_p].

    Returns:
    - The shortest distance between the line and the point.
    """
    # Step 1: Unpack inputs
    a = Matrix(line_point)
    d = Matrix(line_direction)
    p = Matrix(point)

    print(f"Step 1: Inputs")
    print(f"Line: r = ({a[0]}, {a[1]}, {a[2]}) + λ({d[0]}, {d[1]}, {d[2]})")
    print(f"Point: p = ({p[0]}, {p[1]}, {p[2]})")
    print()

    # Step 2: Represent a general point on the line
    λ = symbols('λ')
    r = a + λ * d
    print(f"Step 2: General point on the line:")
    print(f"r = ({r[0]}, {r[1]}, {r[2]})")
    print()

    # Step 3: Compute the vector (p - r)
    p_minus_r = p - r
    print(f"Step 3: Compute the vector (p - r):")
    print(f"p - r = ({p[0]} - {r[0]}, {p[1]} - {r[1]}, {p[2]} - {r[2]}) = ({p_minus_r[0]}, {p_minus_r[1]}, {p_minus_r[2]})")
    print()

    # Step 4: Dot (p - r) with d and set it to zero
    eq = p_minus_r.dot(d)
    print(f"Step 4: Set (p - r) · d = 0:")
    print(f"({p_minus_r[0]}){d[0]} + ({p_minus_r[1]}){d[1]} + ({p_minus_r[2]}){d[2]} = 0")
    print(f"Simplify: {eq} = 0")
    print()

    # Step 5: Solve for λ
    λ_value = solve(eq, λ)[0]
    print(f"Step 5: Solve for λ: λ = {λ_value}")
    print()

    # Step 6: Substitute λ back into r to find the closest point
    closest_point = r.subs(λ, λ_value)
    print(f"Step 6: Substitute λ back into r to find the closest point:")
    print(f"Closest point on the line: ({closest_point[0]}, {closest_point[1]}, {closest_point[2]})")
    print()

    # Step 7: Compute the distance between p and the closest point
    distance_vector = p - closest_point
    distance = distance_vector.norm()
    print(f"Step 7: Compute the distance between p and the closest point:")
    print(f"Distance = sqrt(({distance_vector[0]})^2 + ({distance_vector[1]})^2 + ({distance_vector[2]})^2) = {distance}")
    print()

    return distance


def shortest_distance_point_plane(plane_coeffs, plane_constant, point):
    """
    Find the shortest distance between a point and a plane.

    Parameters:
    - plane_coeffs: Coefficients of the plane equation [a, b, c].
    - plane_constant: The constant d in the plane equation ax + by + cz + d = 0.
    - point: The point [x_p, y_p, z_p].

    Returns:
    - The shortest distance between the point and the plane.
    """
    # Step 1: Unpack inputs
    a, b, c = plane_coeffs
    d = plane_constant
    x_p, y_p, z_p = point

    print(f"Step 1: Inputs")
    print(f"Plane equation: {a}x + {b}y + {c}z + {d} = 0")
    print(f"Point: ({x_p}, {y_p}, {z_p})")
    print()

    # Step 2: Plug the point into the plane equation
    numerator = abs(a * x_p + b * y_p + c * z_p + d)
    print(f"Step 2: Plug the point into the plane equation:")
    print(f"|{a}*{x_p} + {b}*{y_p} + {c}*{z_p} + {d}| = |{a*x_p + b*y_p + c*z_p + d}| = {numerator}")
    print()

    # Step 3: Compute the norm of the normal vector
    denominator = sqrt(a**2 + b**2 + c**2)
    print(f"Step 3: Compute the norm of the normal vector:")
    print(f"sqrt({a}^2 + {b}^2 + {c}^2) = sqrt({a**2 + b**2 + c**2}) = {denominator}")
    print()

    # Step 4: Compute the shortest distance
    distance = numerator / denominator
    print(f"Step 4: Compute the shortest distance:")
    print(f"Distance = |{a}*{x_p} + {b}*{y_p} + {c}*{z_p} + {d}| / sqrt({a}^2 + {b}^2 + {c}^2) = {numerator} / {denominator} = {distance}")
    print()

    return distance


def shortest_distance_parallel_planes(plane1_coeffs, plane1_constant, plane2_coeffs, plane2_constant):
    """
    Find the shortest distance between two parallel planes.

    Parameters:
    - plane1_coeffs: Coefficients of the first plane equation [a1, b1, c1].
    - plane1_constant: The constant d1 in the first plane equation a1x + b1y + c1z + d1 = 0.
    - plane2_coeffs: Coefficients of the second plane equation [a2, b2, c2].
    - plane2_constant: The constant d2 in the second plane equation a2x + b2y + c2z + d2 = 0.

    Returns:
    - The shortest distance between the two parallel planes.
    """
    # Step 1: Unpack inputs
    a1, b1, c1 = plane1_coeffs
    d1 = plane1_constant
    a2, b2, c2 = plane2_coeffs
    d2 = plane2_constant

    print(f"Step 1: Inputs")
    print(f"Plane 1: {a1}x + {b1}y + {c1}z + {d1} = 0")
    print(f"Plane 2: {a2}x + {b2}y + {c2}z + {d2} = 0")
    print()

    # Step 2: Check if the planes are parallel
    if (a1 / a2) != (b1 / b2) or (b1 / b2) != (c1 / c2):
        print("The planes are not parallel. Cannot compute the shortest distance.")
        return None

    print("Step 2: The planes are parallel.")
    print()

    # Step 3: Find a point on Plane 1
    # Let x = 0 and y = 0, then solve for z
    if c1 != 0:
        z = (-d1) / c1
        point_on_plane1 = [0, 0, z]
    elif b1 != 0:
        y = (-d1) / b1
        point_on_plane1 = [0, y, 0]
    elif a1 != 0:
        x = (-d1) / a1
        point_on_plane1 = [x, 0, 0]
    else:
        print("Invalid plane equation.")
        return None

    print(f"Step 3: Find a point on Plane 1:")
    print("Let x and y be 0 then solve for z. Or in another order if coefficient of z is 0")
    print(f"Point on Plane 1: ({point_on_plane1[0]}, {point_on_plane1[1]}, {point_on_plane1[2]})")
    print()

    # Step 4: Compute the distance between the point and Plane 2
    distance = shortest_distance_point_plane(plane2_coeffs, plane2_constant, point_on_plane1)
    print(f"Step 4: Compute the distance between the point and Plane 2:")
    print(f"Distance = {distance}")
    print()

    return distance


# User input handling
print("Enter 1 to find the intersection of two lines: ")
print("Enter 2 to find the intersection of a line and a plane: ")
print("Enter 3 to find the intersection of two planes: ")
print("Enter 4 to find the shortest distance between two lines: ")
print("Enter 5 to find the shortest distance between a line and a plane: ")
print("Enter 6 to find the shortest distance between a plane and a point")
print("Enter 7 to find the shortest distance between two parallel planes")
choice = int(input())

if choice == 1:
    l1a = list(map(int, input("Enter the position vector of line 1 (space-separated): ").split()))
    l1b = list(map(int, input("Enter the direction vector of line 1 (space-separated): ").split()))
    l2a = list(map(int, input("Enter the position vector of line 2 (space-separated): ").split()))
    l2b = list(map(int, input("Enter the direction vector of line 2 (space-separated): ").split()))

    find_intersection_LL(l1a, l1b, l2a, l2b)
elif choice == 2:
    line_point = list(map(int, input("Enter the position vector of the line (space-separated): ").split()))
    line_direction = list(map(int, input("Enter the direction vector of the line (space-separated): ").split()))
    plane_normal = list(map(int, input("Enter the plane coefficients (space-separated, as ax + by + cz = d): ").split()))
    plane_constant = int(input("Enter the constant term of the plane equation (d): "))

    print(find_intersection_PL(plane_normal, plane_constant, line_point, line_direction))
elif choice == 3:
    plane1_normal = list(map(int, input("Enter the first plane coefficients (space-separated, as ax + by + cz = d): ").split())) # Normal vector of the first plane
    plane1_constant = int(input("Enter the constant term of the first plane equation (d): "))        # d1 in the plane equation a1x + b1y + c1z = d1
    plane2_normal = list(map(int, input("Enter the second plane coefficients (space-separated, as ax + by + cz = d): ").split())) # Normal vector of the second plane
    plane2_constant = int(input("Enter the constant term of the second plane equation (d): "))       # d2 in the plane equation a2x + b2y + c2z = d2

    print(find_intersection_PP(plane1_normal, plane1_constant, plane2_normal, plane2_constant))

elif choice == 4:
    l1a = list(map(int, input("Enter the position vector of line 1 (space-separated): ").split()))
    l1b = list(map(int, input("Enter the direction vector of line 1 (space-separated): ").split()))
    l2a = list(map(int, input("Enter the position vector of line 2 (space-separated): ").split()))
    l2b = list(map(int, input("Enter the direction vector of line 2 (space-separated): ").split()))
    print(f"Shortest distance: {shortest_distance_between_lines(l1a,l1b,l2a,l2b)}")

elif choice == 5:
    l1a = list(map(int, input("Enter the position vector of line 1 (space-separated): ").split()))
    l1b = list(map(int, input("Enter the direction vector of line 1 (space-separated): ").split()))
    p = list(map(int, input("Enter the vector of the point (space-separated): ").split()))
    print(f"Shortest distance: {shortest_distance_line_point(l1a, l1b, p)}")

elif choice == 6:
    plane1_normal = list(map(int, input("Enter the first plane coefficients (space-separated, as ax + by + cz = d): ").split())) # Normal vector of the first plane
    plane1_constant = int(input("Enter the constant term of the first plane equation (d): "))*-1        # d1 in the plane equation a1x + b1y + c1z = d1
    p = list(map(int, input("Enter the vector of the point (space-separated): ").split()))
    print(f"Shortest distance: {shortest_distance_point_plane(plane1_normal, plane1_constant, p)}")

elif choice == 7:   
    plane1_normal = list(map(int, input("Enter the first plane coefficients (space-separated, as ax + by + cz = d): ").split())) # Normal vector of the first plane
    plane1_constant = int(input("Enter the constant term of the first plane equation (d): "))*-1        # d1 in the plane equation a1x + b1y + c1z = d1
    plane2_normal = list(map(int, input("Enter the second plane coefficients (space-separated, as ax + by + cz = d): ").split())) # Normal vector of the second plane
    plane2_constant = int(input("Enter the constant term of the second plane equation (d): "))*-1       # d2 in the plane equation a2x + b2y + c2z = d2 
    print(f"Shortest distance: {shortest_distance_parallel_planes(plane1_normal, plane1_constant, plane2_normal, plane2_constant)}")

    
