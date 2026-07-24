import sys
import matplotlib.pyplot as plt
import re
import numpy as np

class HeightMap:
    lower_bounds = []
    upper_bounds = []
    cells = [[]]

    def __init__(self, lower_bounds, upper_bounds, cells):
        self.lower_bounds = lower_bounds
        self.upper_bounds = upper_bounds
        self.cells = cells

    """
    Display the heightmap using imshow
    """

    def save(self, title):

        fig, ax = plt.subplots()

        im = plt.imshow(self.cells, cmap="plasma")

        # Colorbar settings
        cbar = fig.colorbar(im)
        cbar.set_label("Height (m)", size=16)
        cbar.ax.tick_params(labelsize=16)

        # Tick settings
        plt.yticks([])
        plt.xticks([])

        plt.savefig(f"{title}.png")


"""
Returns a hmap as a numpy matrix from a given file

@args
file : An already opened file (i.e. using open())
"""


def heightmap_from_file(file, skip_first):
    row_list = []

    lower_bounds = [0.0, 0.0]
    upper_bounds = [0.0, 0.0]

    for line in file:
        # Extract the bounds from the first line
        if not skip_first:
            skip_first = True

            # remove the non numbers
            # stripped_line = line.strip("bnds:").strip("[").strip("]")
            #

            split = line.split("[")

            low = re.sub("[^0-9\\.,]", "", split[1])

            high = re.sub("[^0-9\\.,]", "", split[2])

            low = low.split(",")
            high = high.split(",")

            # Extract the number and set the bounds
            for i in range(2):
                lower_bounds[i] = float(low[i])
                upper_bounds[i] = float(high[i])

            continue

        # Create the row
        row = []

        # Extract each value in the row
        for value in line.strip().split(","):
            # Ensure the value is not empty
            if value != "":
                if value == "0.0":
                    row.append(np.nan)
                else:
                    row.append(np.float64(value))

        row_list.append(row)

    cells = np.matrix(row_list)

    return HeightMap(lower_bounds, upper_bounds, cells)


if __name__ == "__main__":
  

    load_filename = "../../../../Data/test_dumps/map/hmap_0.txt"
    save_filename = "../../../../Data/test_dumps/map/hmap_0"


    hmap = heightmap_from_file(open(load_filename), False)
    hmap.save(save_filename)
