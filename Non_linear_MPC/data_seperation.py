# Open the input file
with open('/home/radar/Documents/MPC/Model_Predictive_control/Non_linear_MPC/1714111628_0.5.txt', 'r') as f:
    lines = f.readlines()

# Open the output file
with open('curve_0.5.txt', 'w') as f:
    # Iterate over each line in the input file
    for line in lines:
        # Split the line by commas
        parts = line.strip().split(',')
        # Extract the 2nd and 3rd values and format them
        # new_line = '[' + parts[1] + ',' + parts[2] + '],'
        new_line = '['+parts[1] + ',' + parts[2] + '],'
        # Write the new line to the output file
        f.write(new_line + '\n')
