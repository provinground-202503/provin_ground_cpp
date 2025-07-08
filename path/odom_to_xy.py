import csv

def process_odometry_csv(input_filename="odometry_data.csv", output_filename="odometry_xy.csv"):
    """
    Reads a CSV file with 'timestamp', 'x_position', 'y_position' columns
    and creates a new CSV file with only 'X' and 'Y' columns.
    """
    try:
        with open(input_filename, mode='r', newline='') as infile, \
             open(output_filename, mode='w', newline='') as outfile:

            reader = csv.reader(infile)
            writer = csv.writer(outfile)

            # Read the header from the input file
            header = next(reader)
            
            # Find the indices of 'x_position' and 'y_position'
            try:
                x_index = header.index('x_position')
                y_index = header.index('y_position')
            except ValueError:
                print(f"Error: Input CSV must contain 'x_position' and 'y_position' columns in its header.")
                return

            # Write the new header to the output file
            writer.writerow(['X', 'Y'])

            # Iterate over the remaining rows and write only X and Y
            for row in reader:
                if len(row) > max(x_index, y_index): # Ensure row has enough columns
                    writer.writerow([row[x_index], row[y_index]])
                else:
                    print(f"Warning: Skipping malformed row: {row}")

        print(f"Successfully processed '{input_filename}' and created '{output_filename}'.")

    except FileNotFoundError:
        print(f"Error: The file '{input_filename}' was not found. Please make sure it's in the same directory as the script.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    process_odometry_csv()