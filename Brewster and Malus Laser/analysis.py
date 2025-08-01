import matplotlib.pyplot as plt
import numpy as np
import os
import re
import scipy.optimize as opt
from scipy.stats import norm

# -------------------------------- general utils -------------------------------

def uncertainty(data, confidence):
    uncertainty = np.std(data, axis=0) / np.sqrt(len(data))
    uncertainty *= (norm.ppf(0.5 + confidence / 2) - norm.ppf(0.5 - confidence / 2)) / 2
    return uncertainty

def fit_and_plot(
        data,
        f,
        plot, # should we plot the data, or just fit and return the parameters?
        show, # True: show the plot, False: save it to a file
        title = "",
        curve_name = "",
        xlabel = "", ylabel = "",
        p0=None,
        ignore_x_vals=lambda _: False, # x values to ignore when fitting
        additional_plot=[], # additional curves to plot
        notes=[],
        main_colour='#f2a900',
        vlines=[] # vertical lines to draw on the plot
    ):
    x = np.arange(data.shape[1])
    mean = np.mean(data, axis=0)
    sigma = uncertainty(data, 0.68)
    if len(data) == 1:
        sigma = np.ones_like(mean)

    cf_x, cf_mean, cf_sigma = [], [], []
    for xi, meani, sigmai in zip(x, mean, sigma):
        if not ignore_x_vals(xi):
            cf_x.append(xi)
            cf_mean.append(meani)
            cf_sigma.append(sigmai)
    cf_x, cf_mean, cf_sigma = np.array(cf_x), np.array(cf_mean), np.array(cf_sigma)


    popt, _ = opt.curve_fit(f, cf_x, cf_mean, sigma = cf_sigma, maxfev=10000, p0=p0)
    # print(np.sqrt(np.diag(cov)))

    # clever way to avoid correlation between parameters in single parameter
    # error estimates: re-fit for each parameter
    perr = []
    for i in range(len(popt)):
        f_i = lambda x, pi: f(x, *popt[:i], pi, *popt[i+1:])
        _, cov_i = opt.curve_fit(f_i, cf_x, cf_mean, sigma = cf_sigma, maxfev=10000, p0=popt[i])
        perr.append(np.sqrt(cov_i[0][0]))


    fit_y = f(cf_x, *popt)
    chi_squared = np.sum(((cf_mean - fit_y) ** 2) / (cf_sigma ** 2))
    rsquared = 1 - sum((cf_mean - fit_y) ** 2) / sum((cf_mean - np.mean(cf_mean)) ** 2)

    if plot:
        fit_y = f(x, *popt)
        fig, axs = plt.subplots(2, 1, sharex=True, gridspec_kw={'height_ratios': [3, 2]})

        # draw black dotted line at 0
        axs[1].axhline(0, color='black', linestyle='--', linewidth=1)
        axs[1].axhline(1, color='black', linestyle='dotted', linewidth=1)
        axs[1].axhline(-1, color='black', linestyle='dotted', linewidth=1)

        for vline in vlines:
            x_pos = vline(popt)
            axs[0].axvline(x_pos, color='black', linestyle='--', linewidth=1)
            axs[1].axvline(x_pos, color='black', linestyle='--', linewidth=1)

        confidences = [0.68, 0.95]
        colours = ['#2d494f', '#5f7c82']
        axs[0].plot(x, mean, color=colours[0])
        for i in range(len(confidences) - 1, -1, -1):
            confidence = confidences[i]
            bar_height = uncertainty(data, confidences[i])

            axs[0].fill_between(x, mean - bar_height, mean + bar_height, color=colours[i], alpha=0.5, label=f"{confidence:.0%} confidence", edgecolor='none')

        params = {'label': curve_name + " fit", 'color': main_colour, 'linewidth': 1.5}
        axs[0].plot(x, fit_y, **params)
        axs[1].plot(x, (fit_y - mean) / sigma, marker='o', markersize=3, **params)

        axs[1].set_xlabel(xlabel, fontsize=12)
        axs[0].set_ylabel(ylabel, fontsize=12)
        axs[1].set_ylabel("Curve Residuals [σ]",fontsize=12)

        for f, name, colour in additional_plot:
            params = {'label': name, 'color': colour, 'linewidth': 1, 'linestyle': '--'}
            axs[0].plot(x, f(x, *popt), **params)

        axs[0].legend()
        axs[0].set_title(title, fontsize=12)

        text = []
        text.append(("", 12))
        text.append((f"For {curve_name} fit:", 12))
        text.append((f"    χ² = {chi_squared:.2f}", 12))
        text.append((f"    R² = {rsquared:.4f}", 12))
        text.append(("", 12))
        for note in notes:
            text.append((note, 10))

        # write some text at the bottom
        total_adj = 0.05
        for line, size in reversed(text):
            fig.text(0.1, total_adj, line, fontsize=size)
            total_adj += 0.02 * size / 8
        total_adj += 0.05
        fig.subplots_adjust(bottom=total_adj)

        # set the aspect ratio
        fig.set_figwidth(10)
        fig.set_figheight(7)


        # tune out y ticks on the residuals plot
        y_ticks = [-5, -3, -1, 0, 1, 3, 5]
        axs[1].set_ylim(-y_ticks[-1], y_ticks[-1])
        axs[1].set_yticks(y_ticks)
        axs[1].set_yticklabels(map(lambda x: f"{int(x)}σ", y_ticks))

        x_max = int(np.max(x))
        x_ticks = np.linspace(0, x_max, 15)
        for ax in axs:
            ax.set_xticks(x_ticks)
            ax.set_xticklabels(map(lambda xi: f"{int(xi * 360 / 400)}°", x_ticks))

        if show:
            plt.show()
        else:
            filename = re.sub(r'\s+', '_', title)
            filename = re.sub(r'\W+', '', filename)
            iter = 0
            while os.path.exists(f'{filename}_{iter}.png'): iter += 1
            plt.savefig(f'{filename}_{iter}.png', dpi=500)

        plt.close()

    # return popt, chi_squared, rsquared
    return zip(popt, perr), chi_squared, rsquared




# ---------------------- analysis part 1: malus' law ---------------------------
#
# each data in this series is generated by
# 1. shining a laser through
# 2. a polarising filter mounted stepper motor
# 3. into a photodiode
# 4. and recording the intensity of the light as the motor rotates the filter


data = []
for filename in os.listdir('data'):
    if filename.startswith('polarised_10000_'):
        data.append(np.loadtxt('data/' + filename))

data = np.array(data)

# we should have exactly 400 data points per run, corresponding to 1 full rotation
assert data.shape[1] == 400

def f_polarisation(x, a, b, theta):
    return a * np.cos(2 * np.pi * x / 400 + theta) ** 2 + b

((pol_a, pol_a_err), (pol_b, pol_b_err), (pol_theta, pol_theta_err)), chi_squared, rsquared = fit_and_plot(data, f_polarisation, True, False,
    f'Intensity vs Angle of Polarising Filter\n{len(data)} repetitions',
    '$\\cos^2$',
    'Angle of Polarising Filter',
    'Photodiode Intensity\n[ADC reading]')

# mod pi instead of 2pi, since that's the period of cos^2
print(f'Fit parameters: a = {pol_a}, b = {pol_b}, theta = {pol_theta % np.pi}')

angle = (-pol_theta) % np.pi
angle = 400 * angle / (2 * np.pi)
print(f'Max is at {angle} and {angle + 200} steps')

# ---------------------- analysis part 2: brewster's angle ---------------------

data = []
for filename in os.listdir('data'):
    if filename.startswith('transparent_10000_'):
        data.append(np.loadtxt('data/' + filename)[::-1])
data = np.array(data)

# we should have exactly 100 data points per run, corresponding to 90 degrees
assert data.shape[1] == 100

def theta_t(theta_i, n_1, n_2):
    return np.arcsin(np.sin(theta_i) * n_1 / n_2)

def r_s(theta_i, n_1, n_2):
    t_t = theta_t(theta_i, n_1, n_2)
    return ((n_1 * np.cos(theta_i) - n_2 * np.cos(t_t)) / (n_1 * np.cos(theta_i) + n_2 * np.cos(t_t))) ** 2

def r_p(theta_i, n_1, n_2):
    t_t = theta_t(theta_i, n_1, n_2)
    return ((n_1 * np.cos(t_t) - n_2 * np.cos(theta_i)) / (n_1 * np.cos(t_t) + n_2 * np.cos(theta_i))) ** 2

def transmittance(theta_i, n_1, n_2):
    r = (r_s(theta_i, n_1, n_2) + r_p(theta_i, n_1, n_2)) / 2
    t = 1 - r
    return t

# from the Fresnel equations:
def f(x, a, b, n_ratio):
    n_1 = 1
    n_2 = n_ratio
    theta_i = x * 2 * np.pi / 400
    t1 = transmittance(theta_i, n_1, n_2) # transmittance from the air into the glass
    t2 = transmittance(theta_t(theta_i, n_1, n_2), n_2, n_1) # transmittance from the glass back into the air
    return a * t1 * t2  + b # ignores possibility for multiple internal reflections

def f_rs(x, a, b, n_ratio):
    n_1 = 1
    n_2 = n_ratio
    theta_i = x * 2 * np.pi / 400
    return a * r_s(theta_i, n_1, n_2) + b

def f_rp(x, a, b, n_ratio):
    n_1 = 1
    n_2 = n_ratio
    theta_i = x * 2 * np.pi / 400
    return a * r_p(theta_i, n_1, n_2) + b




((a, a_err), (b, b_err), (n_ratio, n_ratio_err)), chi_squared, rsquared = fit_and_plot(data, f, False, False, p0=[300, 100, 1.5])
print('n2/n1 =', n_ratio, '±', n_ratio_err)

mrp = np.argmin(f_rp(np.arange(100), a, b, n_ratio)) * 360 / 400
# Approximation of error on the minimum:
mrp_min = mrp
mrp_max = mrp
for bits in range(1 << 3):
    a_ =       a       + a_err * ((bits & 1) * 2 - 1)
    b_ =       b       + b_err * ((bits >> 1 & 1) * 2 - 1)
    n_ratio_ = n_ratio + n_ratio_err * ((bits >> 2 & 1) * 2 - 1)
    mrp_ = np.argmin(f_rp(np.arange(100), a_, b_, n_ratio_)) * 360 / 400
    if mrp_ < mrp_min: mrp_min = mrp_
    if mrp_ > mrp_max: mrp_max = mrp_
mrp_err = (mrp_max - mrp_min) / 2

fit_and_plot(data, f, True, False,
        title = f'Transmittance as a function of Incident Angle\n{len(data)} repetitions',
        curve_name = 'Fresnel Equation Total Transmittance',
        xlabel = 'Angle between Laser and Glass Slide Normal',
        ylabel = 'Photodiode Intensity\n[ADC reading]',
        p0=[a, b, n_ratio],
        ignore_x_vals=lambda x: x > 83 * 400 / 360,
        main_colour='#FAA916',
        additional_plot = [
            (f_rs, 'S-Polarised Reflectivity (scaled)', '#EA638C'),
            (f_rp, 'P-Polarised Reflectivity (scaled)', '#B33C86'),
        ],
        notes=[
            "Angles greater than 83° were ignored from fit calculations, as they",
            " allowed an internal reflection to pass out of the side of the glass",
            " slide and into the photodiode.",
            "",
            "Best fit with $\\frac{n_2}{n_1} = " + f"{n_ratio:.2f} \\pm {n_ratio_err:.2f}$",
            "Predicting a minimum P-Reflectivity at $\\theta = " + f"{mrp:.1f}° \\pm {mrp_err:.1f}°$"
        ],
        vlines=[
            lambda popt: np.argmin(f_rp(np.arange(100), *popt))
        ]
    )


# print(f'Fit parameters: a = {a}, b = {b}, n_1 = {n_1}, n_2 = {n_2}')

# ---- analysis part 3: sensitivity of polarisation reading to timing change ---

data = {}
for filename in os.listdir('data'):
    if filename.startswith('polarised_'):
        timing = int(filename.split('_')[1])
        data[timing / 1e6] = np.loadtxt('data/' + filename)

timings = list(data.keys())
timings.sort()

results = []

for timing in timings:
    ((a, a_err), (b, b_err), (theta, theta_err)), _, r_squared = fit_and_plot(np.array([data[timing]]), f_polarisation, False, False, p0=[pol_a, pol_b, pol_theta])
    results.append((a, b, theta % np.pi, r_squared))

results = np.array(results)

fig, axs = plt.subplots(len(results[0]), 1, sharex=True)

for i in range(len(results[0])):
    y = results[:, i]
    axs[i].set_xscale('log')
    axs[i].plot(timings, y, marker='o', markersize=3)
    axs[i].set_ylabel(['Best fit for A', 'Best fit for B', 'Best fit for $\\theta_0$', 'R²'][i], fontsize=10)


axs[-1].set_xlabel('Delay between Stepper Motor step and Photodiode reading [s]')

fig.suptitle('Sensitivity of Malus\' Law Curve Fit to Read Timing\nBest fit for $A + B \\cos^2(\\theta + \\theta_0)$ and $R^2$ vs Delay Time', fontsize=12)

# set the aspect ratio
fig.set_figwidth(10)
fig.set_figheight(5)

filename = 'timing'
iter = 0
while os.path.exists(f'{filename}_{iter}.png'): iter += 1
plt.savefig(f'{filename}_{iter}.png', dpi=500)

