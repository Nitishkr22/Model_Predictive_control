# Open the input file
with open('/home/radar/Documents/MPC/Model_Predictive_control/Non_linear_MPC/waypoints_lidar_gps_ud.txt', 'r') as f:
    lines = f.readlines()

# Open the output file
with open('lidar_xy.txt', 'w') as f:
    # Iterate over each line in the input file
    for line in lines:
        # Split the line by commas
        parts = line.strip().split(',')
        # Extract the 2nd and 3rd values and format them
        # new_line = '[' + parts[1] + ',' + parts[2] + '],'
        new_line = parts[0] + ',' + parts[1] + '],'
        # Write the new line to the output file
        f.write(new_line + '\n')
