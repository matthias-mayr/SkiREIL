#!/usr/bin/env python3
# encoding: utf-8
#|
#|    Copyright Inria July 2017
#|    This project has received funding from the European Research Council (ERC) under
#|    the European Union's Horizon 2020 research and innovation programme (grant
#|    agreement No 637972) - see http://www.resibots.eu
#|
#|    Contributor(s):
#|      - Matthias Mayr (matthias.mayr@cs.lth.se)
#|      - Konstantinos Chatzilygeroudis (konstantinos.chatzilygeroudis@inria.fr)
#|      - Rituraj Kaushik (rituraj.kaushik@inria.fr)
#|      - Roberto Rama (bertoski@gmail.com)
#|
#|
#|    This software is governed by the CeCILL-C license under French law and
#|    abiding by the rules of distribution of free software.  You can  use,
#|    modify and/ or redistribute the software under the terms of the CeCILL-C
#|    license as circulated by CEA, CNRS and INRIA at the following URL
#|    "http://www.cecill.info".
#|
#|    As a counterpart to the access to the source code and  rights to copy,
#|    modify and redistribute granted by the license, users are provided only
#|    with a limited warranty  and the software's author,  the holder of the
#|    economic rights,  and the successive licensors  have only  limited
#|    liability.
#|
#|    In this respect, the user's attention is drawn to the risks associated
#|    with loading,  using,  modifying and/or developing or reproducing the
#|    software by the user in light of its specific status of free software,
#|    that may mean  that it is complicated to manipulate,  and  that  also
#|    therefore means  that it is reserved for developers  and  experienced
#|    professionals having in-depth computer knowledge. Users are therefore
#|    encouraged to load and test the software's suitability as regards their
#|    requirements in conditions enabling the security of their systems and/or
#|    data to be ensured and,  more generally, to use and operate it in the
#|    same conditions as regards security.
#|
#|    The fact that you are presently reading this means that you have had
#|    knowledge of the CeCILL-C license and that you accept its terms.
#|
# glob allows us to list the files that match a pattern
import glob
# pylab will be useful later
from pylab import *

from palettable.colorbrewer.qualitative import Set3_12
colors = Set3_12.mpl_colors

params = {
    'axes.labelsize': 8,
    'font.size': 8,
    'legend.fontsize': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'text.usetex': False,
    'figure.figsize': [4.5, 4.5],
    'pdf.fonttype': 42,
    'ps.fonttype': 42
}
rcParams.update(params)

def load_data(base_dir, flip_y_sign = True):
    f_list = glob.glob(base_dir + '/*/optimizations/*_output_pareto.csv')
    data = []

    for f in f_list:
        loaded = np.loadtxt(f, skiprows=1, delimiter=',')

        if len(loaded.shape) == 1:
            loaded.shape = (1, loaded.size)
            print("Only single point in:", f)
        loaded = loaded[:, -3:-1]
        data.append(loaded)
        if flip_y_sign:
            for el in data[-1]:
                el[1] = -el[1]
    return data


# load our data
data_peg = load_data('/tmp/skireil/learning_skills/peg')
data_push = load_data('/tmp/skireil/learning_skills/push')


def plot_all(data, name='peg_insertion', y_label='Insertion Reward', normalize=True, horizontal=True):
    # start plotting
    fig, ax = plt.subplots()

    for i in range(len(data)):
        argx = np.argsort(data[i][:,0], axis=0)
        if len(argx) is 1:
            x = data[i][[0],0]
            y = data[i][[0],1]
        else:
            x = data[i][argx,0]
            y = data[i][argx,1]

        if normalize:
            if len(argx) is 1:
                if x.min(axis=0):
                    x = x / x.min(axis=0)
                if y.min(axis=0):
                    y = y / y.min(axis=0)
            else:
                x = (x - x.min(axis=0)) / (x.max(axis=0) - x.min(axis=0))
                y = (y - y.min(axis=0)) / (y.max(axis=0) - y.min(axis=0))

        if horizontal:
            if len(argx) is 1:
                red_shape = x.shape[0]
                xx = np.zeros(((x.shape[0]),))
                yy = np.zeros(((y.shape[0]),))
            else:
                red_shape = x.shape[0]-1
                xx = np.zeros((2*(x.shape[0]-1),))
                yy = np.zeros((2*(y.shape[0]-1),))
            for k in range(red_shape):
                xx[2*k] = x[k]
                yy[2*k] = y[k]
                if len(argx) > 1:
                    xx[2*k+1] = x[k+1]
                    yy[2*k+1] = y[k]


            xx[2*k] = x[k]
            yy[2*k] = y[k]

            x = xx
            y = yy

        c = [colors[i%len(colors)]]*x.shape[0]

        plt.plot(x, y, '--', color=colors[i%len(colors)])
        plt.scatter(x, y, c=c, edgecolors='black')
        # break

    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.get_xaxis().tick_bottom()
    ax.get_yaxis().tick_left()
    ax.tick_params(axis='x', direction='out')
    ax.tick_params(axis='y', length=0)

    ax.grid(axis='x', color="0.9", linestyle='-', linewidth=1)
    ax.grid(axis='y', color="0.9", linestyle='-', linewidth=1)
    ax.set_axisbelow(True)

    ax.set_ylabel(y_label)
    ax.set_xlabel('Force')

    plt.tight_layout()

    if normalize:
        plt.savefig(name + "_task_pareto_output.png", dpi=300)
        plt.savefig(name + "_task_pareto_output.pdf", dpi=300)
        plt.savefig(name + "_task_pareto_output.svg")
    else:
        plt.savefig(name + "_task_pareto_output_raw.png", dpi=300)
        plt.savefig(name + "_task_pareto_output_raw.pdf", dpi=300)
        plt.savefig(name + "_task_pareto_output_raw.svg")

# plot_all(data_peg, 'peg_insertion', y_label='Insertion Reward')
# plot_all(data_push, 'push', y_label='Push Reward')


plot_all(data_peg, 'peg_insertion', y_label='Insertion Reward', normalize=False)
plot_all(data_push, 'push', y_label='Push Reward', normalize=False)
