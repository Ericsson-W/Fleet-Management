import numpy as np

class PID:

    def __init__(self, controller_parameters: dict):
        self.k_p: float     = controller_parameters['p_gain']   # proportional gain
        self.k_d: float     = controller_parameters['d_gain']   # derivative gain   
        self.k_i: float     = controller_parameters['i_gain']   # integral gain
        
        self.antiwindup: bool      = controller_parameters['antiwindup'] 
        self.max_error_integral: float = controller_parameters['max_error_integral']

        self.past_error: float      = 0.0
        self.error_sum: float       = 0.0
        self.curr_error_sign: int   = None
        self.prev_error_sign: int   = None


    # TODO: this should not take delta_t but rather internally determine the time
    # that has passed since control_step() was called
    def control_step(self, e, delta_t):
        """ Takes error 'e' and provides control action 'u'. 
            'delta_t' should be time interval since the last control step (used for estimating derivative term). 
        """

        self.error_sign     = np.sign(e)
        self.error_sum      += e

        if self.antiwindup: self.antiwindup_measures()

        prop_term: float    = self.k_p * e
        der_term: float     = self.k_d * ((e - self.past_error) / delta_t) # TODO: DIVIDE BY DT?
        int_term: float     = self.k_i * self.error_sum

        u: float            = prop_term + der_term + int_term

        self.past_error         = e
        self.prev_error_sign    = self.error_sign

        return u


    def antiwindup_measures(self):

        # If error changes sign, reset error integral
        if (self.prev_error_sign is not None) and (self.prev_error_sign != self.error_sign):
            self.error_sum = 0.0

        # Limits absolute value of error sum to be below the set threshold
        if (np.abs(self.error_sum) > self.max_error_integral):
            self.error_sum = self.error_sign * self.max_error_integral