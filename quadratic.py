import math
# formula = input("Enter the quadratic: ")

# a,b,c = formula.split("+")
a = float(input("Enter integer a: "))
b = float(input("Enter integer b: "))
c = float(input("Enter integer c: "))
if b**2 > 4*a*c:
    answer1 = (-b+math.sqrt((b**2)-4*a*c))/(2*a)
    answer2 =  (-b-math.sqrt((b**2)-4*a*c))/(2*a )
    print(f"x = {answer1}")
    print(f"x = {answer2}")
elif b**2 == 4*a*c:
    answer1 = -b/(2*a)
    answer2 = answer1
    print(f"x = {answer1}")
    print(f"x = {answer2}")
else:
    print("No solution")
