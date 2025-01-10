
class PID:
    """Class to implement a PID controller
    """

    def __init__(self,k_p:float,k_i:float,k_d:float,distance:float):
        """Initialize the object

        Parameters
        ----------
        k_p : float
            Constant P for the system
        k_i : float
            Constant I for the system
        k_d : float
            Constant D for the system
        distance : float
            Desired distance to mantain
        """

        self.k_p=k_p
        self.k_i=k_i
        self.k_d=k_d
        self.input=distance
        self.error_i=0
        self.error_prev=0

    def error(self,act_distance:float):
        """Calculates the errors and restarts the error I, if necessary

        Parameters
        ----------
        act_distance : float
            Actual distance to calculate the errors
        """

        self.error_act= self.input-act_distance
        self.error_i+=self.error_act
        if self.error_i > 1 or self.error_i < -1: #If the error I gets to big, restart it to mantain stable the system
            self.error_i=0
        self.error_d=self.error_act-self.error_prev
    
    def activate(self,act_distance:float)->float:
        """Calculates the PID output and calls the error calculation too

        Parameters
        ----------
        act_distance : float
            Actual distance to calculate the errors

        Returns
        -------
        float
            Final controlled Angular velocity.
        """

        self.error(act_distance) #calculate the errors
        pid_calc=self.k_p*self.error_act+self.k_i*self.error_i+self.k_d*self.error_d #calculate the final output
        self.error_prev=self.error_act
        return pid_calc


