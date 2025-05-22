import os
from weis import weis_main

import warnings
warnings.filterwarnings("ignore")


## File management
run_dir = os.path.dirname( os.path.realpath(__file__) )
fname_wt_input = os.path.join(run_dir,"IEA-22-280-RWT-Semi.yaml")
fname_modeling_options = os.path.join(run_dir, "modeling_options_dlc_1p1_2.yaml")
fname_analysis_options = os.path.join(run_dir, "analysis_options_2.yaml")

wt_opt, modeling_options, opt_options = weis_main(fname_wt_input, 
                                                 fname_modeling_options, 
                                                 fname_analysis_options)
