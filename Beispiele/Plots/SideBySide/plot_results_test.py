import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from plots import *

# Define the function and its derivative
def f(x):
    return -1/3 * x**3 - 0.2 * x + 0.53333333

def f_prime(x):
    return -x**2 - 0.2

# Define the range of x
x = np.linspace(-1, 1, 400)


plot = Plot(xlabel=r"$\beta_p$", ylabel="penalty", width=2.5, ratio=1)
plot.ax.plot(x, f(x), color=TUM_BLUE)

plt.tight_layout(pad=0.2)
plt.savefig("penalty_filter.pdf")

plot = Plot(xlabel=r"$\beta_p$", ylabel=r"$\nabla_{\beta_p} \text{penalty}$", width=2.5, ratio=1)
plot.ax.plot(x, f(x), color=TUM_BLUE)

plt.tight_layout(pad=0.2)
plt.savefig("penalty_filter_grad.pdf")
