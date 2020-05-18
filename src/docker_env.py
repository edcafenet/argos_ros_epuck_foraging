#!/usr/bin/env python
import docker
import re

class DockerEnv:

    client = docker.from_env()
    container = None
    RobotID = None

    @property
    def JSQueryCodePath(self):
        return self._JSQueryCodePath
    @JSQueryCodePath.setter
    def JSQueryCodePath(self, value):
        self._JSQueryCodePath = value
        
    @property
    def SmartContractAddress(self):
        return self._SmartContractAddress
    @SmartContractAddress.setter
    def SmartContractAddress(self, value):
        self._SmartContractAddress = value

    @property
    def SmartContractPath(self):
        return self._SmartContractPath
    @SmartContractPath.setter
    def SmartContractPath(self, value):
        self._SmartContractPath = value

    @property
    def SmartContractParameter(self):
        return self._SmartContractParameter
    @SmartContractParameter.setter
    def SmartContractParameter(self, value):
        self._SmartContractParameter = value

    @property
    def SmartContractOutput(self):
        return self._SmartContractOutput
    @SmartContractParameter.setter
    def SmartContractOutput(self, value):
        self._SmartContractOutput = value
    
    def __init__(self, RobotROSNameSpace):
        self.RobotID = self.getRobotID(RobotROSNameSpace)
        self.getContainer()
        
        self._JSQueryCodePath = "/root/js/check_proof.js"
        self._SmartContractPath = "/root/smart_contracts/MerkleTreeComplete.sol"
        self._SmartContractAddress = "0xFF37a57B8D373518aBE222Db1077eD9A968a5FDf"
        self._SmartContractParameter = "green"
        
    def getContainer(self):
        DockerComposerID = int(self.RobotID) + 1
        self.container = self.client.containers.get("dockergethnetwork_eth_"\
                                                    + str(DockerComposerID))
    def getRobotID(self, RobotROSNameSpace):
        RobotROSNameSpace = re.sub("[/]", "", RobotROSNameSpace)
        RobotID = RobotROSNameSpace.replace('bot','')
        return RobotID
        
    def execRun(self):
        command = "node" + " " +\
                  self.JSQueryCodePath + " " +\
                  self.SmartContractPath + " "+\
                  self.SmartContractAddress + " "+\
                  self.SmartContractParameter

        return self.container.exec_run(command)

    def evalOutput(self):
        self._SmartContractOutput = self.execRun()
        if(self._SmartContractOutput[1] == 'true'):
            return True
        else:
            return False 
