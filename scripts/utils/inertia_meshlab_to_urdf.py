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
from __future__ import print_function


def print_xml(s, m, v, com, d):
    # Calculation according to:
    # http://gazebosim.org/tutorials?tut=inertia

    com = [x / s for x in com]

    for key in d.keys():
        d[key] = d[key] / s**2
        d[key] = d[key] / v
        d[key] = d[key] * m

    print("<inertial>")
    print("  <mass value=\"{}\" />".format(m))
    print("  <origin xyz=\"{} {} {}\" rpy=\"0 0 0\" />".format(com[0], com[1], com[2]))
    print("  <inertia ixx=\"{ixx}\" ixy=\"{ixy}\" ixz=\"{ixz}\" iyy=\"{iyy}\" iyz=\"{iyz}\" izz=\"{izz}\" />".format(**d))
    print("</inertial>")
    print()

    return


if __name__ == '__main__':
    # Scaling factor. How much bigger is the model than in reality?
    # If it was designed with OpenSCAD, s is usually 1000
    s = 1000

    m = 1.5
    v = 1575000.0
    com = (-200.000000, -50.000000, 35.000000)
    d = {'ixx': 2611874560, 'ixy': 1968753024, 'ixz': 0.0001, 'iyy': 8518120448, 'iyz': 0.0001, 'izz': 9843745792}
    print_xml(s, m, v, com, d)
