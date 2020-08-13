import time

####################################################################################################
#### Sync the stepped simulation with the wall-clock ###############################################
####################################################################################################
#### Arguments #####################################################################################
#### - i (Integer)			    current step of the simulation #########################
#### - start_time (Timestamp)		    timestamp of the simulation start ######################
#### - timestep (Float)			    desired, wall-clock timestep to render the simulation ##
####################################################################################################
def sync(i, start_time, timestep):
	elapsed = time.time()-start_time
	if elapsed < i*(timestep): time.sleep(i*(timestep)-elapsed)	
