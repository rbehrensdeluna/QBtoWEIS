import os
from weis import weis_main

## File management
<<<<<<< HEAD
run_dir = os.path.dirname( os.path.realpath(__file__) )
fname_wt_input = os.path.join(run_dir, "..", "00_setup", "ref_turbines", "nrel5mw-spar_oc3.yaml")
fname_modeling_options = os.path.join(run_dir, "modeling_options.yaml")
fname_analysis_options = os.path.join(run_dir, "analysis_options.yaml")
=======
run_dir                 = os.path.dirname( os.path.realpath(__file__) )
fname_wt_input          = run_dir + os.sep + "nrel5mw-spar_oc3.yaml"
fname_modeling_options  = run_dir + os.sep + 'modeling_options_test.yaml'
fname_analysis_options  = run_dir + os.sep + 'analysis_options_noopt.yaml'
>>>>>>> test-develop

wt_opt, modeling_options, opt_options = weis_main(fname_wt_input, 
                                                 fname_modeling_options, 
                                                 fname_analysis_options)
