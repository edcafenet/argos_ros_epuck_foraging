#!/usr/bin/env python
from merkletools import *
import hashlib
import more_itertools as mit
from auxiliary_functions import *
import time
import sys
import rospy
import pickle
import json
from std_msgs.msg import String

class SwarmMerkleTree:

    @property
    def working_root(self):
        return self._working_root
    @working_root.setter
    def working_root(self, value):
        self._working_root = value

    @property
    def completed_merkle(self):
        return self._completed_merkle
    @completed_merkle.setter
    def completed_merkle(self, value):
        self._completed_merkle = value

    def __init__(self, data):
        # Tree constructed step by step with actions of the robot
        self._completed_merkle = []
        received_json_object = json.loads(data)

        self.memory_merkle = MerkleTools(hash_type="sha256")
        for item in received_json_object:
            self.memory_merkle.add_leaf(item['hash'])

        self.memory_merkle.make_tree()
        self.working_root = self.memory_merkle.get_merkle_root()
        self.init_completed_merkle()

    def init_completed_merkle(self):
        for i in xrange(self.memory_merkle.get_leaf_count()):
            leaf = dict(id=i, completed=False, hash_action='', hash_input='', received=False, from_robot=-1)
            self.completed_merkle.append(leaf)

    def get_proof(self, leaf_index):
        return self.memory_merkle.get_proof(leaf_index)

    def get_leaf(self, leaf_index):
        return self.memory_merkle.get_leaf(leaf_index)

    def generate_hash(self, value):
        return hashlib.sha256(value).hexdigest()

    def check(self, hash_action, hash_sensor_input, leaf_index):
        working_merkle = MerkleTools(hash_type="sha256")
        working_leaf = self.generate_hash(hash_action + hash_sensor_input)
        working_proof = self.get_proof(leaf_index)

        # Sequential code
        leafs_completed = list(mit.locate(self.completed_merkle, pred=lambda d: d["completed"] == True))
        if leafs_completed == []:
            last_leaf_completed = -1
        else:
            last_leaf_completed = max(leafs_completed)

        # Sequential conditional
        if (leaf_index >= 0 and leaf_index < self.memory_merkle.get_leaf_count() and leaf_index == last_leaf_completed+1):
            if (not self.completed_merkle[leaf_index]['completed']):
                if(working_merkle.validate_proof(working_proof, working_leaf, self.working_root)):
                    self.completed_merkle[leaf_index]['hash_action'] = hash_action
                    self.completed_merkle[leaf_index]['hash_input'] = hash_sensor_input
                    return True
                else:
                    return False
            else:
                return False
        else:
            return False

    def complete_leaf(self, leaf_index):
        self.completed_merkle[leaf_index]['completed'] = True

    def is_complete_merkle_done(self):
        return all(d['completed'] for d in self.completed_merkle)

    def get_number_of_leafs(self):
        return self.memory_merkle.get_leaf_count()
