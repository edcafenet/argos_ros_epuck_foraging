#!/usr/bin/env python
from merkletools import *
import hashlib
import sha3
import more_itertools as mit
from auxiliary_functions import *
from maze import Maze
import time
import sys

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
        
    def __init__(self, num_leafs):
        # Tree constructed step by step with actions of the robot
        self._completed_merkle = []
        self.memory_merkle = MerkleTools(hash_type="sha3_256")      
        
        leaf_colors = ["green", "red", "blue", "yellow", "magenta", "cyan", "white", "orange"]
        self.memory_merkle = MerkleTools(hash_type="sha3_256")      
        
        # Adding leafs according to the agreed solution
        for x in xrange(num_leafs):
            self.memory_merkle.add_leaf(leaf_colors[x], True)

        self.memory_merkle.make_tree()
        self.working_root = self.memory_merkle.get_merkle_root() 
        self.init_completed_merkle()

    def init_completed_merkle(self):        
        for i in xrange(self.memory_merkle.get_leaf_count()):
            leaf = dict(id=i, completed=False, hash='', received=False, from_robot=-1)
            self.completed_merkle.append(leaf)
            
    def get_proof(self, leaf_index):
        return self.memory_merkle.get_proof(leaf_index)

    def get_leaf(self, leaf_index):
        return self.memory_merkle.get_leaf(leaf_index)
    
    def generate_hash_leaf(self, leaf):
        return hashlib.sha3_256(leaf).hexdigest()
    
    def check(self, leaf_value, leaf_index):
        working_merkle = MerkleTools(hash_type="sha3_256")
        working_leaf = self.generate_hash_leaf(leaf_value)
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
                    self.completed_merkle[leaf_index]['hash'] = working_leaf
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
