
def getBottlesData():
    # Open the file and read its contents
    with open("BOTTLESDATA.txt", "r") as file:
        data = file.readlines()

    # Initialize empty lists to store the data
    letter_list = []
    color_list = []
    shape_list = []

    # Iterate over the lines in the file
    for line in data:
        # Split the line into a list using the comma as a delimiter
        line_data = line.strip().split(",")
        
        # Extract the letter, color, and shape from the line
        letter = line_data[0].strip()[1:-1] # remove the leading and trailing quote
        color = line_data[1].strip()
        shape = line_data[2].strip()

        # Append the letter, color, and shape to the appropriate lists
        letter_list.append(letter)
        color_list.append(color)
        shape_list.append(shape)

    # Print the lists to verify the data
    print("Letters: ",letter_list)
    print("Colors: ",color_list)
    print("Shapes: ",shape_list)

    return letter_list,color_list,shape_list
pass


if __name__ == '__main__':
    getBottlesData()
    