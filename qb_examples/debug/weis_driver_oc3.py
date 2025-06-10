import os
from weis import weis_main

## File management
run_dir = os.path.dirname( os.path.realpath(__file__) )
fname_wt_input = os.path.join(run_dir,"MED15-300_v16.2.0.yaml")
fname_modeling_options = os.path.join(run_dir, "modeling_options_PRECOMP.yaml")
fname_analysis_options = os.path.join(run_dir, "analysis_options_PRECOMP.yaml")

wt_opt, modeling_options, opt_options = weis_main(fname_wt_input, 
                                                 fname_modeling_options, 
                                                 fname_analysis_options)
