import time

def sync_sim(i, start_time, timestep):
	elapsed = time.time()-start_time
	if elapsed < i*(timestep): time.sleep(i*(timestep)-elapsed)	
