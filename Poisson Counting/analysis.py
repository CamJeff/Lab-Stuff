import numpy as np
import os
import matplotlib.pyplot as plt
from scipy.stats import poisson, norm
import scipy.optimize as opt
import sys


def load_data(filename):
    values = []
    with open(filename, "r") as file:
        for line in file:
            line = line.strip()
            if not line.startswith('count (1000ms): '):
                print('discarding abnormal reading[', line, ']')
                continue

            values.append(int(line.split(':')[1].strip()))

    return np.array(values)


def plot(data, num_splits, plot = True):

    mean = np.mean(data)
    print('mean:', np.mean(data))
    data = data[:len(data) // num_splits * num_splits] # truncate data to be divisible by num_splits
    print('truncated data to', len(data), 'points')

    print('sub-sections:', num_splits)

    num_bins = np.max(data) + 1
    x = np.unique(data)

    observations = []

    for replica in np.array_split(data, num_splits):

        # get the counts of values in the replica
        counts = np.unique(replica, return_counts=True)
        # print(', '.join(map(lambda x: f'{x:02}', counts[0])))
        # print(', '.join(map(lambda x: f'{x:02}', counts[1])))

        # set 0 for any missing values (present in x, but not in replica)
        for i in range(len(x)):
            if x[i] not in counts[0]:
                counts = (np.insert(counts[0], i, x[i]), np.insert(counts[1], i, 0))

        observations.append(counts[1])

    def mean_and_uncertainty(confidence):
        observed_mean = np.mean(observations, axis=0)
        observed_mean_uncertainty = np.std(observations, axis=0) / np.sqrt(num_splits)
        observed_mean_uncertainty *= norm.ppf(0.5 + confidence / 2) - norm.ppf(0.5 - confidence / 2)) / 2
        return observed_mean, observed_mean_uncertainty

    fig, axs = None, None
    if plot:
        fig, axs = plt.subplots(2, 1, sharex=True, gridspec_kw={'height_ratios': [3, 2]})
        fig.subplots_adjust(hspace=0.2)

        # draw black dotted line at 0
        axs[1].axhline(0, color='black', linestyle='--', linewidth=1)
        axs[1].axhline(1, color='black', linestyle='dotted', linewidth=1)
        axs[1].axhline(-1, color='black', linestyle='dotted', linewidth=1)


        confidences = [0.68, 0.95]
        colours = ['#2d494f', '#5f7c82']
        for i in range(len(confidences) - 1, -1, -1):
            confidence = confidences[i]
            observed_mean, observed_mean_uncertainty = mean_and_uncertainty(confidence)

            bar_width = 2 + 8 * (1 - confidence)

            axs[0].errorbar(x, observed_mean,
                         yerr=observed_mean_uncertainty,
                         elinewidth=bar_width,
                         capsize=2,
                         linestyle='None',
                         color=colours[i],
                         label=f"Observed Data ({confidence:.0%} confidence)")

    residuals_zero, residuals_one = mean_and_uncertainty(0.68)

    # fit a poisson distribution to the data, with mean taken from the total data

    p_mean, p_scale = opt.curve_fit(lambda x, mean, scale: poisson.pmf(x, mean) * scale, x, residuals_zero,sigma=residuals_one)[0]
    poisson_y = poisson.pmf(x, p_mean) * p_scale

    if plot:
        assert axs is not None # there's gotta be some better way to typecheck this

        params = {'label': "Poisson fit", 'color': '#f2a900', 'linewidth': 1.5}
        axs[0].plot(x, poisson_y, **params)
        axs[1].plot(x, (poisson_y - residuals_zero) / residuals_one, marker='o', markersize=3, **params)

    # fit a gaussian distribution to the data, with mean and std taken from the total data
    gauss_mean, gauss_std, gauss_scale = opt.curve_fit(lambda x, mean, std, scale: norm.pdf(x, mean, std) * scale, x, residuals_zero,sigma=residuals_one)[0]
    gauss_y = norm.pdf(x, gauss_mean, gauss_std) * gauss_scale
    params = {'label': "Gaussian fit", 'color': '#de5823', 'linewidth': 1.5}

    if plot:
        assert axs is not None
        axs[0].plot(x, gauss_y, **params)
        axs[1].plot(x, (gauss_y - residuals_zero) / residuals_one, marker='o', markersize=3, **params)

        axs[1].set_xlabel("Clicks per 1000ms", fontsize=12)
        axs[0].set_ylabel("Number of Observations", fontsize=12)
        axs[1].set_ylabel("Curve Residuals (σ)", fontsize=12)

        # set x ticks to be integers
        for ax in axs:
            ax.set_xticks(x)
            ax.set_xticklabels(map(str, x))

        axs[0].legend()

        axs[0].set_title(f"Clicks per 1000ms\n{len(data)} data points split into {num_splits} replicas with {len(data) // num_splits} points each", fontsize=12)

    # write the following text down at the bottom, with 8 point font
    text = []

    # chi-square test
    chi_squared_poisson = (1 / len(x)) * np.sum(((residuals_zero - poisson_y) ** 2) / (residuals_one ** 2))
    chi_squared_gauss   = (1 / len(x)) * np.sum(((residuals_zero - gauss_y) ** 2) / (residuals_one ** 2))

    if plot:
        assert fig is not None
        assert axs is not None
        text.append((f"For Poisson fit: χ² = {chi_squared_poisson:.2f}", 12))
        text.append((f"For Gaussian fit: χ² = {chi_squared_gauss:.2f}", 12))
        text.append(("", 12))

        ommitted_vals = set(list(range(num_bins))).difference(set(x))
        ommitted_text = ', '.join(list(map(str, ommitted_vals)) + ['≥' + str(num_bins)])
        ommitted_text = f"Note 1: Click counts [{ommitted_text}] were never observed, and have been ommitted from the plot, curve fits, and chi-square calculation."
        text.append((ommitted_text, 8))


        # predicted total counts
        poisson_total = p_scale
        gauss_total = gauss_scale
        text.append((f"Note 2: Poisson CDF predicts {poisson_total:.1f} observations per replica ({100 * poisson_total * num_splits / len(data):.1f}% of actual amount)", 8))
        text.append((f"Note 3: Gaussian CDF predicts {gauss_total:.1f} observations per replica ({100 * gauss_total * num_splits / len(data):.1f}% of actual amount)", 8))


        # write some text at the bottom
        total_adj = 0.05
        for line, size in reversed(text):
            fig.text(0.1, total_adj, line, fontsize=size)
            total_adj += 0.02 * size / 8
        total_adj += 0.05
        print(total_adj)
        fig.subplots_adjust(bottom=total_adj)

        # set the aspect ratio
        fig.set_figwidth(10)
        fig.set_figheight(7)


        # tune out y ticks on the residuals plot
        y_ticks = [-5, -3, -1, 0, 1, 3, 5]
        print(y_ticks)
        axs[1].set_ylim(-y_ticks[-1], y_ticks[-1])
        axs[1].set_yticks(y_ticks)
        axs[1].set_yticklabels(map(lambda x: f"{int(x)}σ", y_ticks))

        i = 0
        filename_base = f'output_m{int(mean)}_c{len(data)}_s{num_splits}'
        filename = f'{filename_base}.png'
        while os.path.exists(filename):
            i += 1
            filename = f'{filename_base}_{i}.png'

        plt.savefig(filename, dpi=300)

    return chi_squared_poisson, chi_squared_gauss



if len(sys.argv) != 3 or not os.path.exists(sys.argv[1]) or sys.argv[2] not in ['plot', 'chi_range']:

    print('Usage: python3 analysis.py <filename> < single | chi_range >')

    if not os.path.exists(sys.argv[1]):
        print(f'file "{sys.argv[1]}" does not exist')
    if sys.argv[2] not in ['single', 'chi_range']:
        print(f'invalid mode: "{sys.argv[2]}"')

    exit()

filename = sys.argv[1]
mode = sys.argv[2]

data = load_data(filename)

print(len(data), 'data points loaded')


# split the data into num_splits distinct sub-ranges, estimating the uncertainty
# on each bin by the spread of the data in that sub-range


if mode == 'single':
    # plot the data with a single replica size

    num_splits = int(len(data) ** 0.4) # tends to get a good balance between
                                       # enough data points in each bin, and
                                       # enough bins to estimate a good
                                       # uncertainty

    plot(data, num_splits, plot = True)
    exit()


assert mode == 'chi_range'
# plot the chi-squared values of the best fits for both Poisson and Gaussian


x = []
y1 = []
y2 = []
xmin = 2 # minimum number of data points per replica
for num_splits in range(1, len(data)):
    xc = len(data) // num_splits
    if len(x) > 0 and x[-1] == xc:
        continue
    if xc < xmin:
        continue

    print('num_splits:', num_splits)
    print('len(data):', len(data))

    a, b, = plot(data, num_splits, plot = False)
    print('chi_squared_poisson:', a)
    print('chi_squared_gauss:', b)
    if a == np.inf or b == np.inf:
        continue
    x.append(xc)
    y1.append(a)
    y2.append(b)


plt.gcf().set_figwidth(10)
plt.plot(x, y1, label='χ² of best Poisson fit', marker='o', markersize=3, color='#f2a900')
plt.plot(x, y2, label='χ² of best Gaussian fit', marker='o', markersize=3, color='#de5823')

# draw a horizontal line at 1.39, most probable χ² value for 2 degrees of freedom
plt.axhline(1.39, color='#f2a900', linestyle='--', linewidth=1)
plt.text(len(data) - 10, 1.39 - 0.1,
        'χ² = 1.39, most, probable χ² value\nfor 2 degrees of freedom',
        fontsize=8,
        horizontalalignment='right',
        verticalalignment='top')

plt.axhline(2.37 + 0.5, color='#de5823', linestyle='--', linewidth=1)
plt.text(len(data) - 10, 2.37 + 0.5,
    'χ² = 2.37 most probable χ² value\nfor 3 degrees of freedom',
    fontsize=8,
    horizontalalignment='right',
    verticalalignment='bottom')

plt.xlabel('Number of data points per Replica')
plt.ylabel('χ²')
plt.xscale('log')
plt.yscale('log')

# set the ticks to be integers
plt.xlim(xmin, len(data))
plt.ylim(0.5, max(y2) * 1.5)

xticks = [5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000]
xticks = [x for x in xticks if x < len(data)] + [len(data)]
plt.xticks(xticks, list(map(str, map(int, xticks))))

yticks = [0.5, 1, 2, 3, 5, 10, 20, 50, 100]
plt.yticks(yticks, list(map(lambda y: f'{y:.1f}', yticks)))

plt.title('χ² of best Poisson and Gaussian fits, as a function of the number of data points per Replica')

plt.legend()


output_filename = f'output_chi_squared.png'
i = 0
while os.path.exists(output_filename):
    i += 1
    output_filename = f'output_chi_squared_{i}.png'
plt.savefig(output_filename, dpi=300)

# NOTES FROM Talking with Prof. Ryan
# - Error bars are probably too big, check that out
# - Don't normalise data, keep batch size and number of batches ("Replicas") in the report
# - Actually fit both Poisson and Gaussian distributions, give them a fighting chance
# - Don't use datapoints for which we don't have a count
