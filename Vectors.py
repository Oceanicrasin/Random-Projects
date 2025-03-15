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

# User input handling
choice = int(input("Enter choice (1 to find intersection): "))

if choice == 1:
    l1a = list(map(int, input("Enter the position vector of line 1 (space-separated): ").split()))
    l1b = list(map(int, input("Enter the direction vector of line 1 (space-separated): ").split()))
    l2a = list(map(int, input("Enter the position vector of line 2 (space-separated): ").split()))
    l2b = list(map(int, input("Enter the direction vector of line 2 (space-separated): ").split()))

    find_intersection(l1a, l1b, l2a, l2b)
