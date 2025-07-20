import os
from weis import weis_main

import warnings
warnings.filterwarnings("ignore")

# This example shows how to run QBtoWEIS from the last iteration of a provided sql file. 
# The code will use the design varibales of the last iteration and start a fresh WEIS run from there. There is no information concerning the simplex (COBYLA) or gradients (SLSQP) from the previous run

## File management
run_dir = os.path.dirname( os.path.realpath(__file__) )
fname_wt_input = os.path.join(run_dir,"IEA-22-280-RWT-Semi.yaml")
fname_modeling_options = os.path.join(run_dir, "modeling_options_dlc_1p1.yaml")
fname_analysis_options = os.path.join(run_dir, "analysis_options_2.yaml")

wt_opt, modeling_options, opt_options = weis_main(fname_wt_input, 
                                                 fname_modeling_options, 
                                                 fname_analysis_options)
