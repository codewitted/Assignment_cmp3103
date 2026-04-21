# task1.py

# Function to calculate the factorial of a number

def factorial(n):
    """
    Calculate factorial using recursion.
    :param n: Integer, the number to calculate the factorial for.
    :return: Integer, factorial of the number.
    """
    if n < 0:
        raise ValueError("Factorial is not defined for negative numbers.")
    elif n == 0:
        return 1  # base case
    else:
        return n * factorial(n - 1)  # recursive call

# Main function to execute the logic

def main():
    number = int(input("Enter a non-negative integer: "))  # prompt user input
    try:
        result = factorial(number)
        print(f'The factorial of {number} is {result}.')  # display result
    except ValueError as e:
        print(e)  # handle exceptions

if __name__ == '__main__':
    main()  # entry point of the program