"""Script with all the needed classes to implement a Fuzzy System in a desired behaviour, 
with some restrictions, 
such as only such as shapes of membership functions.

Author:Lesly Guerrero
Date: November 25, 2023.
"""

#Import math libraries
from itertools import product
import numpy as np

#STEP 1
class MembershipFunctions:
    """Class to implement Membership Functions for the Fuzzy system
    """
    def __init__(self,inputs:list,outputs:list):
        """Initialize the object

        Args:
            inputs (list): List of dicts containing the inputs, 
            with its 3 Membership values (In the inputs, 
            Working with trapezes at the outer fuzzy sets, 
            and triangles at the center.)
            outputs (list): List of dicts containing the outputs, 
            with its 3 Membership values (In the outputs, 
            working with triangles.)
        """
        #lists of dicts containing everything in format: 
        # {"FRS":{"near":[-np.inf,0.25,0.5],"medium":[0.25,0.5,0.75],"far":[0.25,0.75,np.inf]},"BRS":{"near":[-np.inf,0.25,0.5],"medium":[0.25,0.5,0.75],"far":[0.25,0.75,np.inf]}}
            
        self.mf={"inputs": inputs,"outputs":outputs}

#STEP 2
class RuleBase:
    """Class to generate the rule base for a fuzzy system
    """
    def __init__(self,MembershipFunc:MembershipFunctions):
        """Initialize the object

        Args:
            MembershipFunc (MembershipFunctions): Membership Functions of the system
        """
        self.rule_base=[]
        self.Mf=MembershipFunc
        self.rule={}
        self.permited_outputs=set()
        values=tuple()
        for key in self.Mf["inputs"].keys():
            values+=(self.Mf["inputs"][key].keys(),)
            self.rule[key]=0
        self.i_values=product(*values)

        for key in self.Mf["outputs"].keys():
            self.rule[key]=0
            for element in self.Mf["outputs"][key].keys():
                self.permited_outputs.add(element)

        for element in self.i_values:
            i=0
            for key in self.Mf["inputs"].keys():
                self.rule[key]=element[i]
                i+=1
            self.rule_base.append(self.rule.copy())
        self.no_of_rules=len(self.rule_base)

    def set_rules(self,output_rules:list):
        """Set the users output rules for the rule buse

        Args:
            output_rules (list): List pf lists containing each output rule

        Raises:
            Exception: Invalid output (If a word is miswritten, it can not be saved in the rule base)
            Exception: Error dif len (Every list with a rule needs to be of the same len)
        """
        #output_rules list of lists containing the output value for each oupt in order
        if len(output_rules) == self.no_of_rules:
            i=0
            for rule in self.rule_base:
                j=0
                for key in self.Mf["outputs"].keys():
                    if output_rules[i][j] in self.permited_outputs:
                        rule[key]=output_rules[i][j]
                        j+=1
                    else:
                        
                        raise Exception("Invalid output")
                i+=1
                        
        else:
            raise Exception("Error dif len")
        

class Fuzzifier:
    """Main class of the fuzzy system
    """
    def __init__(self,crisp_inputs:dict):
        """Initialize the object

        Args:
            crisp_inputs (dict): Name of the input, and its value
        """
        self.inputs=crisp_inputs
        self.values=[]
        self.check={}
        self.final_outputs={}
        for key in self.inputs.keys():
            self.check[key]=[]

    def fuzzify(self,MembFuct:MembershipFunctions,rule_base:RuleBase):
        """Main function that Fuzzify crisp inputs, 
        Determines which rules fire and calculate firing strengths, 
        makes the defuzzification.

        Args:
            MembFuct (MembershipFunctions): Membership functions of the fuzzy system
            rule_base (RuleBase): Rule base of the fuzzy system
        """

        #STEP 3
        #get ranges where the inputs are
        for i in self.inputs.keys():
            for key in MembFuct["inputs"].keys():
                for value in MembFuct["inputs"][key].keys():
                    if i == key:
                        minn=min(MembFuct["inputs"][key][value])
                        maxx= max(MembFuct["inputs"][key][value])
                        if self.inputs[i] >= minn and self.inputs[i] <= maxx:
                            self.check[i].append({value:0})
        
        #set values for those ranges
        for key in self.check.keys():
            for i in range(len(self.check[key])):
                for value in self.check[key][i].keys():
                    if len(self.check[key]) == 1:
                        self.check[key][i][value]=1.0
                    elif len(self.check[key]) > 1:
                        a=MembFuct["inputs"][key][value][0]
                        b=MembFuct["inputs"][key][value][1]
                        c=MembFuct["inputs"][key][value][2]
                        x=self.inputs[key]
                        raising_edge=(x-a)/(b-a)
                        falling_edge=(c-x)/(c-b)
                        if falling_edge <= 1:
                            self.check[key][i][value]= round(falling_edge,2)
                        if raising_edge <= 1:
                            self.check[key][i][value]= round(raising_edge,2)
        
        #STEP 4
        #combinations for checking the rules
        comb=tuple()
        for key in self.check.keys():
            vals=[]
            for i in range(len(self.check[key])):
                vals.append(list(self.check[key][i].keys())[0])
            comb+=(vals,)
        combinations=list(product(*comb))
        inp_rules=[]
        keys=list(self.check.keys())
        for i in range(len(keys)-1):  
            for element in combinations:
                r={keys[i]:element[i],keys[i+1]:element[i+1]}
                inp_rules.append(r)
                    
        #select only the rules that apply
        valid_rules=[]
        for rule in rule_base:
            for element in inp_rules:
                if all (item in rule.items() for item in element.items()):
                    valid_rules.append(rule)


        #collect the fuzzified inputs for each rule
        #firing strengths
        final_inps=[]
        for rule in valid_rules:
            inps=[]
            for key in rule.keys():
                if key in keys:
                    for element in self.check[key]:
                        if list(element.keys())[0] == rule[key]:
                            inps.append(list(element.values())[0])
            final_inps.append(min(inps)) #AND connetive (min)
       

        #STEP 5
        #DEFUZZIFICATION

        for key in MembFuct["outputs"].keys():
            centroids=[]
            for rule in valid_rules:
                centroid=MembFuct["outputs"][key][rule[key]]
                centroids.append(centroid)
            output=(np.dot(np.array(final_inps),np.array(centroids)))/sum(final_inps)
            self.final_outputs[key]=output
          



if __name__ == '__main__':

    inputs={"FRS":{"near":[0,0.25,0.5],"medium":[0.25,0.5,0.75],"far":[0.5,0.75,10]},"BRS":{"near":[0,0.25,0.5],"medium":[0.25,0.5,0.75],"far":[0.5,0.75,10]}}
    outputs={"LinX":{"slow":0.2,"medium":0.4,"fast":0.6},"AngZ":{"right":-0.2,"front":0,"left":0.2}}
    falop=MembershipFunctions(inputs,outputs)
    regles= RuleBase(falop.mf)
    out=[["slow","left"],["slow","left"],["medium","left"],["slow","right"],["medium","front"],["medium","right"],["medium","right"],["medium","right"],["medium","right"]]
    
    regles.set_rules(out)
    #print(falop.mf)
    # print("------------------------------------------")
    #print(regles.rule_base)
    crisp_inp={"FRS":0.6,"BRS":0.3}
    fuzzy=Fuzzifier(crisp_inp)
    fuzzy.fuzzify(falop.mf,regles.rule_base)
    print(fuzzy.check)
    #print(fuzzy.final_outputs)

    print(fuzzy.final_outputs.values())
    linear, angular = fuzzy.final_outputs.values()

    print("Linear velocity set to: ", linear)
    print("Angular velocity set to: ", angular)