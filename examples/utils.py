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

####################################################################################################
#### Convert a string into a boolean ###############################################################
####################################################################################################
#### Arguments #####################################################################################
#### - val (?)                              input value (possibly stirng) to interpret as boolean ##
####################################################################################################
#### Returns #######################################################################################
#### - _ (bool)                             the boolean interpretation of the input value ##########
####################################################################################################
def str2bool(val):
    if isinstance(val, bool): return val
    elif val.lower() in ('yes', 'true', 't', 'y', '1'): return True
    elif val.lower() in ('no', 'false', 'f', 'n', '0'): return False
    else: raise print("[ERROR] in str2bool(), a Boolean value is expected")
