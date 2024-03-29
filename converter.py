def main():
    denary = get_int("Input a value: ")
    binary = str(bin(denary))[2:]
    print(binary)
    print(hex_converter(denary))
    
    
def number_to_letter(n):
    if n > 9:
        if n == 10:
            return "A"
        if n == 11:
            return "B"
        if n == 12:
            return "C" 
        if n == 13:
            return "D"  
        if n == 14:
            return "E"    
        if n == 15:
            return "F"  
    return str(n)    
    
    
def hex_converter(h):
    column1= number_to_letter(h // 16)
    column2 = number_to_letter(h % 16)
    return [column1,column2]
    
def get_int(message=""):
    while True:
        try:
            n = int(input(message))
            return n
        except ValueError:
            print("Invalid input try again")
main()    


   
     
    
