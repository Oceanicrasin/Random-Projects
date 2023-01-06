expression = input("Expression: ")

x, y, z = expression.split()
x = float(x)
z = float(z)

match y:
    case "+":
        print(f"{x+z:.1f}")
    case "-":
        print(f"{x-z:.1f}")
    case "/":
        print(f"{x/z:.1f}")
    case "*":
        print(f"{x*z:.1f}")        
