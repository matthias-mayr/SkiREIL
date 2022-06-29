#!/usr/bin/env python
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
#! /usr/bin/env python

import sys
import signal
import Queue
import time
import textwrap
import pickle
import rospy

from skiros2_skill.ros.skill_layer_interface import SkillLayerInterface
import skiros2_common.ros.utils as utils
from control_msgs.msg import FollowJointTrajectoryGoal
from std_msgs.msg import Float32

from blackdrops.srv import ManageSkirosWorker
from blackdrops.srv import ManageSkirosWorkerRequest, ManageSkirosWorkerResponse
from skiros_integration import skiros_rl_client


class SkirosWorkerManager(object):
    def __init__(self, nr_worker):
        rospy.loginfo("Starting SkiROS Worker Manager with %i workers.", nr_worker)
        self.free_workers = Queue.Queue()
        rospy.Service('/skiros_worker_manager', ManageSkirosWorker, self.worker_cb)
        # This is a singleton
        self.sli = SkillLayerInterface("rl_skill_layer")
        # self.sli.set_monitor_cb(self.monitor_cb)

        rospy.Subscriber("take_back_worker", Float32, self.take_back_worker_cb)
        self.verbose = rospy.get_param("~verbose", True)
        self.robot_name = rospy.get_param("~robot_name", "/bh_robot")
        self.robot_name += "_"
        self.queue_dict = {}
        self.worker_handout_time_dict = {}
        for i in range(1, nr_worker + 1):
            # Change to test_robot for integration test
            robot_name = self.robot_name + str(i)
            rospy.loginfo("Waiting forever for the robot %s to appear.", robot_name)
            while robot_name not in self.sli.agents:
                time.sleep(0.1)
            agent = self.sli.agents[robot_name]
            rospy.loginfo("Robot appeared.")
            msg_queue = Queue.Queue()
            self.queue_dict[robot_name] = msg_queue
            skiros_rl_client.SkirosRlClient(i, msg_queue, agent)
            self.free_workers.put(i)

        time.sleep(2.0)
        self.sli.set_debug(True)
        time.sleep(0.5)

    def take_back_worker_cb(self, min):
        if min <= 0.:
            rospy.loginfo("Duration for taking back worker is 0 or smaller. Specify a positive value")
            return
        dur = rospy.Duration.from_sec(min.data * 60.)
        for key, value in self.worker_handout_time_dict.items():
            if rospy.Time.now() - value > dur:
                rospy.logwarn("Taking back worker number %i", key)
                self.free_workers.put(key)

    # Collects TreeProgress messages from the SkillManagers and passes them to the workers
    def monitor_cb(self, tree_progress):
        # check if robot_name exists in the dictionary
        # add it to this dictionary
        if tree_progress.progress[0].robot in self.queue_dict:
            self.queue_dict[tree_progress.progress[0].robot].put(tree_progress)
        else:
            rospy.logwarn("Received progress msg for robot '%s' that is not known.",
                          tree_progress.progress[0].robot)

    # Worker callback that hands out workers
    def worker_cb(self, req):
        response = ManageSkirosWorkerResponse()
        if req.request:
            if self.free_workers.empty():
                rospy.logwarn("All workers are busy. Waiting.")
                while self.free_workers.empty():
                    time.sleep(0.01)
            response.worker_number = self.free_workers.get()
            self.worker_handout_time_dict[response.worker_number] = rospy.Time.now()
            if self.verbose:
                rospy.loginfo("Handed out worker number: %i", response.worker_number)
        else:
            if self.verbose:
                rospy.loginfo("Got back worker number: %i", req.release)
            self.free_workers.put(req.release)
            if not self.worker_handout_time_dict.pop(req.release, None):
                rospy.logwarn("Got back worker %i which was never handed out.", req.release)
        if self.verbose:
            rospy.loginfo("Number of workers remaining: %i", len(self.free_workers.queue))
        return response


if __name__ == '__main__':
    # Allows to exit upon ctrl-c
    def signal_handler(signal, frame):
        rospy.loginfo("\nProgram exiting gracefully")
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    rospy.loginfo('Starting skiros worker manager')
    rospy.init_node('skiros_worker_manager')
    nr_worker = rospy.get_param('~nr_workers', 1)

    worker_manager = SkirosWorkerManager(nr_worker)
    rospy.loginfo('Setup finished. Started spinning')
    rospy.spin()
