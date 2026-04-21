# Task 3: Improved Code

def calculate_sum(numbers):
    # Initialize sum variable to hold the total
    total = 0
    
    # Iterate over each number in the list
    for num in numbers:
        total += num  # Add the current number to the total
    
    return total  # Return the final sum

# Example usage:
if __name__ == '__main__':
    sample_numbers = [1, 2, 3, 4, 5]
    result = calculate_sum(sample_numbers)  # Calculate the sum of the sample numbers
    print(f'The sum of {sample_numbers} is {result}.')  # Output the result