"""
partially based on https://jwalton.info/Embed-Publication-Matplotlib-Latex/
"""
import matplotlib.pyplot as plt

try:
    from .tum_colors import *
except ImportError:
    from tum_colors import *

import shutil
USE_TEX = bool(shutil.which('latex'))

GOLDEN_RATIO = 1.618

class Figure():

    def __init__(self, **kwargs):

        self.fig = None

        tex_fonts = {
            # Use LaTeX to write all text
            "text.usetex": USE_TEX,
            "font.family": "sans-serif",
            # Use 10pt font in plots, to match 10pt font in document
            # or 8pt to match the caption
            "axes.labelsize": 8,
            "font.size": 8,
            # Make the legend/label fonts a little smaller
            "legend.fontsize": 8,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "text.latex.preamble": "".join([
                r"\usepackage{amsmath}",
                r"\usepackage{bm}"
            ])
        }
        plt.rcParams.update(tex_fonts)

        # plot.show or colab in line display is easier to see
        plt.rcParams["figure.dpi"] = 125

        # Size
        width  = kwargs.get("width", 3.33) # 3.487
        height = kwargs.get("height", width / kwargs.get("ratio", 4/3))
        self.fig = plt.figure(
            figsize=(width, height),
            #linewidth=0.5,
            #edgecolor="black",
            facecolor="white"  # white background in colab
        )


class Plot(Figure):

    def __init__(self, **kwargs):

        super().__init__(**kwargs)
        self.ax = None

        #fig.subplots_adjust(left=.15, bottom=.16, right=.99, top=.97)
        self.ax = plt.axes()
        self.ax.set(
            title=kwargs.get("title"),
            xlabel=kwargs.get("xlabel"),
            ylabel=kwargs.get("ylabel")
        )

        self.ax.grid(color='gray', linestyle=':', linewidth=0.5)

    def add_legend(self, **kwargs):
        """Legend has to be added after plots with labels have been added."""
        self.ax.legend(
            loc=kwargs.get("loc", "upper right"),
            framealpha=kwargs.get("framealpha", 1)
        )

class Plot3D(Figure):

    def __init__(self, **kwargs):

        super().__init__(**kwargs)
        # Size
        width  = kwargs.get("width", 3.33) # 3.487
        height = kwargs.get("height", width / kwargs.get("ratio", 4/3))
        self.fig, self.ax = plt.subplots(
            subplot_kw={"projection": "3d"},
            figsize=(width, height),
            #linewidth=0.5,
            #edgecolor="black",
            facecolor="white"  # white background in colab
        )
        self.ax.set(
            title=kwargs.get("title"),
            xlabel=kwargs.get("xlabel"),
            ylabel=kwargs.get("ylabel")
        )

        # self.ax.grid(color='gray', linestyle=':', linewidth=0.5)

    def add_legend(self, **kwargs):
        """Legend has to be added after plots with labels have been added."""
        self.ax.legend(
            loc=kwargs.get("loc", "upper right"),
            framealpha=kwargs.get("framealpha", 1)
        )