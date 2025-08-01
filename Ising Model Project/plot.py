from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

#----------------------------------------------------------------------
#  DOCUMENTS AND STUFF
#----------------------------------------------------------------------

def load_csv(filename):
    print('loading', filename)
    data = np.genfromtxt(filename, delimiter=',', names=True)
    return data['Temperature'], data['Value'], data['Error']

T, E, E_err = load_csv("Observables Data/energy_vs_temperature.csv")
_, M, M_err = load_csv("Observables Data/magnetization_vs_temperature.csv")
_, C, C_err = load_csv("Observables Data/specific_heat_vs_temperature.csv")
_, X, X_err = load_csv("Observables Data/susceptibility_vs_temperature.csv")

T_C = 2.269185314213022

def magnetisation_ising(T):
    T = np.array(T)
    M = np.zeros_like(T)
    below = T < T_C
    M[below] = (1 - 1 / np.sinh(2 / T[below])**4)**(1/8)
    return M

#----------------------------------------------------------------------
#  PLOTTING
#----------------------------------------------------------------------
plt.style.use('seaborn-v0_8-talk')

def plot_with_fit_and_residuals(
    x, y, yerr, func, ylabel, color, filename,
    p0=None, x_bars=[], y_bars=[]):

    # If no fitting function is provided, only plot the main graph.
    if func is None:
        f = plt.figure(figsize=(10, 4))
        f.suptitle(f"{ylabel.split(' (')[0]} as a Function of Temperature", fontsize=18)
        ax = f.add_subplot(1, 1, 1)
        ax.errorbar(x, y, yerr=yerr, fmt='none', color=color, markersize=6,
                    capsize=3, capthick=1, elinewidth=1)
        ax.set_xlabel("Temperature (K)", fontsize=16)
        ax.set_ylabel(ylabel, fontsize=16)
        ax.grid(True, alpha=0.3)
        for bar_x, bar_args in x_bars:
            ax.axvline(bar_x, **bar_args)
        for bar_y, bar_args in y_bars:
            ax.axhline(bar_y, **bar_args)
        ax.legend(fontsize=14)
    else:
        f = plt.figure(figsize=(10, 7))
        f.suptitle(f"{ylabel.split(' (')[0]} as a Function of Temperature with Residuals", fontsize=18)
        # When a fitting function is provided, attempt to fit to the data.
        func, func_name = func
        if p0 is not None and len(p0) > 0:
            popt, pcov = curve_fit(func, x, y, p0=p0, sigma=yerr, absolute_sigma=True)
            y_fit = func(x, *popt)
        else:
            y_fit = func(x)
        res = y - y_fit

        ax1 = f.add_subplot(1, 2, 1)
        ax1.errorbar(x, y, yerr=yerr, fmt='none', color=color, markersize=6,
                     capsize=3, capthick=1, elinewidth=1)
        ax1.plot(x, y_fit, color='black', linewidth=2, label=func_name)
        ax1.set_xlabel("Temperature (K)", fontsize=16)
        ax1.set_ylabel(ylabel, fontsize=16)
        ax1.grid(True, alpha=0.3)
        for bar_x, bar_args in x_bars:
            ax1.axvline(bar_x, **bar_args)
        for bar_y, bar_args in y_bars:
            ax1.axhline(bar_y, **bar_args)
        ax1.legend(fontsize=14)

        ax2 = f.add_subplot(1, 2, 2)
        ax2.errorbar(x, res, yerr=yerr, fmt='none', color=color, markersize=6,
                     capsize=3, capthick=1, elinewidth=1)
        ax2.axhline(0, color='black', linestyle='--', linewidth=1)
        ax2.set_xlabel("Temperature (K)", fontsize=16)
        ax2.set_ylabel(f"{ylabel.split(' (')[0]} Residuals ({ylabel.split(' (')[1]}", fontsize=16)
        ax2.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.93])
    plt.savefig(filename)
    plt.close(f)


plot_with_fit_and_residuals(T, E, E_err, None, 'Energy (J)', 'red', 'Energy.svg')
plot_with_fit_and_residuals(T, M, M_err, (magnetisation_ising, "expected magnitisation"), 'Magnetisation (A/m)', 'blue', 'Magnetization.svg', p0=[],
    x_bars=[(T_C, {'color': 'black', 'linestyle': '--', 'label': f"$T_C$ = {T_C} K", 'linewidth': 1})],
)

plot_with_fit_and_residuals(T, C, C_err, None, 'Specific Heat (J/K)', 'green', 'SpecificHeat.svg',
    x_bars=[(T_C, {'color': 'black', 'linestyle': '--', 'label': f"$T_C$ = {T_C} K", 'linewidth': 1})],
    y_bars=[(0, {'color': 'black', 'linewidth': 1})],
)
plot_with_fit_and_residuals(T, X, X_err, None, 'Susceptibility [dimensionless]', 'purple', 'Susceptibility.svg',
    x_bars=[(T_C, {'color': 'black', 'linestyle': '--', 'label': f"$T_C$ = {T_C} K", 'linewidth': 1})],
    y_bars=[(0, {'color': 'black', 'linewidth': 1})],
)

from scipy.stats import chi2
M_model = magnetisation_ising(T)
chi2_val = np.sum(((M - M_model) / M_err) ** 2)
dof = len(M)
p_value = 1 - chi2.cdf(chi2_val, dof)
print(f"Chi² (zero-parameter fit for magnetization): {chi2_val:.2f}")
print(f"Degrees of Freedom: {dof}")
print(f"p-value: {p_value*100000}")
