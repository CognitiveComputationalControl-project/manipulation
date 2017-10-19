from .sequence_planner import *
from multiprocessing import Process, Queue

import rospy

class ParallelSequencePlanner(object):
    def __init__(self, hand_goal_service, hand_line_service, n):
        self.sequence_planners = [SequencePlanner(hand_goal_service+'_'+str(i), hand_line_service+'_'+str(i)) for i in range(n)]
        self.n = n

        self.odom_to_robot_pose = None
        self.initial_joint_state = None
        self.collision_env = None

    def run_planner(self, i, job_q, ans_q):
        while True:
            seq = job_q.get()
            ans_q.put(self.sequence_planners[i].plan_sequence(seq, self.odom_to_robot_pose, 
                                                              self.initial_joint_state,
                                                              self.collision_env))

    def plan_sequences(self, seqs, odom_to_robot_pose, initial_joint_state, collision_env):
        self.odom_to_robot_pose = odom_to_robot_pose
        self.initial_joint_state = initial_joint_state
        self.collision_env = collision_env

        job_q = Queue()
        ans_q = Queue()

        for seq in seqs:
            jobs_q.put(seq)        

        processes = [Process(target=run_planner, args=(i, job_q, ans_q)) for i in range(self.n)]
        for p in processes:
            p.start()

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not ans_q.empty():
                for p in processes:
                    p.terminate()
                break

            if job_q.empty():
                for p in processes:
                    p.terminate()
                break

            r.sleep()

        if not ans_q.empty():
            return ans_q.get()
        else:
            return []

