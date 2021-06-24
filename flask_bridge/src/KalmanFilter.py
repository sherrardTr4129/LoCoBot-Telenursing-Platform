# Author: Trevor Sherrard
# Course: Directed Research
# Since: June 25, 2021
# Description: This file contains the class definition for the 1D
#              Kalman Filter used for smoothing individual lidar
#	       bin averages

class KalmanFilter:
    '''Kalman Filter Object Definition'''
    def __init__(self, measurement_err, estimate_err, Q_val):
        self.measurement_err = measurement_err
        self.estimate_err = estimate_err
        self.Q_val = Q_val
        self.last_estimate = 0
        self.current_estimate = None

    def updateEstimate(self, new_measurment):
	"""
        Update internal Kalman filter state estimate based on 
        new measurement. return new measurement estimate.

        params:
	    self
            new_measurment (float): new measurment value

        returns:
            current_estimate (float): current estimate of the observed
                                      state based on new measurment data
	"""
        # update kalman gain
        K = self.estimate_err/(self.estimate_err + self.measurement_err)

        # predict current estimate
        if(self.last_estimate == None):
            self.current_estimate = new_measurment
        else:
            self.current_estimate = self.last_estimate + K * (new_measurment - self.last_estimate)

        # update error estimate
        self.estimate_err = (1.0 - K)*self.estimate_err + abs(self.last_estimate - self.current_estimate)*self.Q_val

        # update last estimate value
        self.last_estimate = self.current_estimate

        return self.current_estimate
