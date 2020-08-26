import time

####################################################################################################
#### Sync the stepped simulation with the wall-clock ###############################################
####################################################################################################
#### Arguments #####################################################################################
#### - i (int)                              current simulation iteration ###########################
#### - start_time (timestamp)               timestamp of the simulation start ######################
#### - timestep (float)                     desired, wall-clock step of the simulation's rendering #
####################################################################################################
def sync(i, start_time, timestep):
    if timestep>.04 or i%(int(1/(24*timestep)))==0:
        elapsed = time.time() - start_time
        if elapsed<(i*timestep): time.sleep(timestep*i - elapsed)

