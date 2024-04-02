# Open the input file
with open('1712040276_0.02.txt', 'r') as f:
    lines = f.readlines()

# Open the output file
with open('output_0.02.txt', 'w') as f:
    # Iterate over each line in the input file
    for line in lines:
        # Split the line by commas
        parts = line.strip().split(',')
        # Extract the 2nd and 3rd values and format them
        new_line = '[' + parts[1] + ',' + parts[2] + '],'
        # Write the new line to the output file
        f.write(new_line + '\n')
