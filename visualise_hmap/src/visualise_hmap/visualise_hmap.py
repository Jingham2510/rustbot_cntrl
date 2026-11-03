import re
import numpy as np
import matplotlib
matplotlib.use("Agg")  # non-interactive backend, faster and safe for multiprocessing
import matplotlib.pyplot as plt
from multiprocessing import Pool
from pathlib import Path
import pandas as pd


class HeightMap:
    def __init__(self, lower_bounds, upper_bounds, cells):
        self.lower_bounds = lower_bounds
        self.upper_bounds = upper_bounds
        self.cells = cells

    def save(self, title):
        """Save the heightmap as an image using imshow."""
        fig, ax = plt.subplots()
        im = ax.imshow(self.cells.T, cmap="plasma")  # transpose so it's the right way round
        cbar = fig.colorbar(im)
        cbar.set_label("Depth (m)", size=16)
        cbar.ax.tick_params(labelsize=16)
        ax.set_xticks([])
        ax.set_yticks([])
        fig.savefig(f"{title}.png", dpi=200)
        plt.close(fig)

    def save_surface(self, title):
        """Save the heightmap as a 3D surface."""
        h, w = self.cells.shape
        x, y = np.meshgrid(np.arange(h), np.arange(w), indexing="ij")
        fig = plt.figure()
        ax = fig.add_subplot(projection="3d")
        ax.plot_surface(x, y, self.cells, vmin=np.nanmin(self.cells) * 2,
                        cmap="terrain", rstride=4, cstride=4)  # subsample for speed
        ax.set_xticks([])
        ax.set_yticks([])
        fig.savefig(f"{title}.png", dpi=200)  # save BEFORE show; no show() in batch runs
        plt.close(fig)


_num_re = re.compile(r"[^0-9\.,]")


def heightmap_from_file(path):
    """Load a heightmap from a text file. First line holds the bounds."""
    with open(path) as f:
        header = f.readline()
        split = header.split("[")
        low = [float(v) for v in _num_re.sub("", split[1]).split(",") if v]
        high = [float(v) for v in _num_re.sub("", split[2]).split(",") if v]

        # Parse the rest in C instead of looping in Python
        cells = pd.read_csv(f, header=None).dropna(axis=1, how="all").to_numpy(dtype=np.float64)

    # Original code treated "0.0" as missing
    cells[cells == 0.0] = np.nan
    return HeightMap(low[:2], high[:2], cells)


def process(no, base="../../../Data/test_dumps/contact_map"):
    hmap = heightmap_from_file(f"{base}/hmap_{no}.txt")
    hmap.save(f"{base}/hmap_{no}")


if __name__ == "__main__":
    with Pool() as pool:
        pool.map(process, range(1, 28), chunksize=8)
