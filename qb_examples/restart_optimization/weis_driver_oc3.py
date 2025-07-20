import os
from weis import weis_main


# This example shows how to restart a QBtoWEIS run from the last iteration of a provided sql file. 
# This code will load the entire sql file and use the inputs/outputs stored in the sql file to fast-track the optimization process. As COBYLA does not store the simplex we re-run the optimization process by prescribing the inputs and outputs from the previou sql file instead of calling QBlade.
# once the final iteration of the previous sql fil is reached the optimization will continue from there as if it was a fresh run. 

# Important note: This only works for identical optimization problem formulations. It is a workaround to be able to run on cluster that constrain wall-clock time or restart optimization runs that are not fully converged at the max_iteration limit or crashed for some other reaseon (pc-shutdown, power outage, etc.).



## File management
run_dir = os.path.dirname( os.path.realpath(__file__) )
fname_wt_input = os.path.join(run_dir,"nrel5mw-spar_oc3.yaml")
fname_modeling_options = os.path.join(run_dir, "modeling_options_dlc_1p6.yaml")
fname_analysis_options = os.path.join(run_dir, "analysis_options_opt.yaml")

wt_opt, modeling_options, opt_options = weis_main(fname_wt_input, 
                                                 fname_modeling_options, 
                                                 fname_analysis_options)
