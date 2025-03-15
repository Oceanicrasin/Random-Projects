import sympy as sym

def find_intersection(a1, b1, a2, b2):
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


from sympy import symbols, Eq, solve

def find_intersection(plane_normal, plane_constant, line_point, line_direction):
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




# User input handling
print("Enter 1 to find the intersection of two lines: ")
print("Enter 2 to find the intersection of a line and a plane: ")
choice = int(input())

if choice == 1:
    l1a = list(map(int, input("Enter the position vector of line 1 (space-separated): ").split()))
    l1b = list(map(int, input("Enter the direction vector of line 1 (space-separated): ").split()))
    l2a = list(map(int, input("Enter the position vector of line 2 (space-separated): ").split()))
    l2b = list(map(int, input("Enter the direction vector of line 2 (space-separated): ").split()))

    find_intersection(l1a, l1b, l2a, l2b)
if choice == 2:
    line_point = list(map(int, input("Enter the position vector of the line (space-separated): ").split()))
    line_direction = list(map(int, input("Enter the direction vector of the line (space-separated): ").split()))
    plane_normal = list(map(int, input("Enter the plane coefficients (space-separated, as ax + by + cz = d): ").split()))
    plane_constant = int(input("Enter the constant term of the plane equation (d): "))

    print(find_intersection(plane_normal, plane_constant, line_point, line_direction))
