
"""Script with all the needed classes to implement a Fuzzy Conext Blend Coordinator, 
with some restrictions, 
such as only 1 FuzzySet per membership function.

Author:Lesly Guerrero
Date: November 25, 2023.
"""
#Import math libraries
from itertools import product
import numpy as np

#STEP 1
class MembershipFunctions:
    """ Class to generate Membership Functions for the fuzzy system
    """
    def __init__(self,inputs:list):
        """Inizialize the object

        Args:
            inputs (list): List of dicts containing inputs, outputs and it's values. Membership values should be trapezoidal
        """
        #lists of dicts containing everything in format: 
        # {"D1":{"low":[0,0.45,0.65]},"D2":{"low":[0,0.45,0.65]}}
            
        self.mf={"inputs": inputs}


class FuzzyCoordinator:
    """Main class of the fuzzy context blend coordinator, with al steps to pass from a crisp input to a crisp output.
    """
    def __init__(self,crisp_inputs:dict,behaviour_outputs:dict):
        """Initialize the object

        Args:
            crisp_inputs (dict): Name of the input, and it's crisp value.
            behaviour_outputs (dict): Name of output of the desired behaviours with a list with it's own crisp outputs.
        """
        #behaviour output in format: {LinX:[OA,REF],AngZ:[OA,REF]}

        self.inputs=crisp_inputs
        self.values=[]
        self.check={}
        self.final_outputs={}
        self.b_outputs=behaviour_outputs
        for key in self.inputs.keys():
            self.check[key]=[]

    def fuzzify(self,MembFuct:MembershipFunctions):
        """Main function, it will Fuzzify crisp outputs, and make the defuzzification

        Args:
            MembFuct (MembershipFunctions): Membership functions of the inputs
        """
        #STEP 3
        #get ramges where the inputs are
        for i in self.inputs.keys():
            for key in MembFuct["inputs"].keys():
                for value in MembFuct["inputs"][key].keys():
                    if i == key:
                        minn=min(MembFuct["inputs"][key][value])
                        maxx= max(MembFuct["inputs"][key][value])
                        if self.inputs[i] >= minn and self.inputs[i] <= maxx:
                            self.check[i].append({value:0})
            if not bool(self.check[i]):
                self.check[i].append({"None":0})
        
        #set values for those ranges
        for key in self.check.keys():
            for i in range(len(self.check[key])):
                for value in self.check[key][i].keys():
                    if value == "None":
                        self.check[key][i][value]=0.0
                    else:
                        a=MembFuct["inputs"][key][value][0]
                        b=MembFuct["inputs"][key][value][1]
                        c=MembFuct["inputs"][key][value][2]
                        x=self.inputs[key]
                        
                        falling_edge=(c-x)/(c-b)
                        if x <= b:
                            self.check[key][i][value]=1.0
                        
                        else:
                            self.check[key][i][value]= round(falling_edge,2)
                        
        #STEP 5
        #DEFUZZIFICATION

        memb_values=[]
        for behaviour in self.check.keys():
            
            for element in self.check[behaviour]:
                val = list(element.values())[0]
                memb_values.append(val)
        
        print(self.check)
        for key in self.b_outputs.keys():
            high_output=np.dot(np.array(memb_values),np.array(self.b_outputs[key]))/sum(memb_values)
            self.final_outputs[key]=high_output
          

if __name__ == '__main__':

    inputs={
    "D1": {
        "near": [0, 1, 2]
    },
    "D2": {
        "near": [0,  1, 2]
    }
}
    falop=MembershipFunctions(inputs)
    
    #print(falop.mf)
    # print("------------------------------------------")
    #print(regles.rule_base)
    crisp_inp={"D1":3.5,"D2":2.5}
    b_outputs={"LinX":[0.31,-0.17],"AngZ":[0.45,0.2]}
    fuzzy=FuzzyCoordinator(crisp_inp,b_outputs)
    fuzzy.fuzzify(falop.mf)
    print(fuzzy.check)
    #print(fuzzy.final_outputs)

    linear, angular = fuzzy.final_outputs.values()

    print("Linear velocity set to: ", linear)
    print("Angular velocity set to: ", angular)