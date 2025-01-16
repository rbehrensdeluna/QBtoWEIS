
"""

Copyright © 2024 Robert Behrens de Luna. All rights reserved.

This software project and all associated files are protected by copyright and may not
be copied, distributed, or modified without the express written permission of the 
copyright holder.

A significant portion of this project relies on code from the WISDEM/WEIS repository (https://github.com/WISDEM/WEIS),
which is licensed under the Apache 2.0 License. The terms of the Apache 2.0 License apply to those parts and can be
found in the LICENSE file. The remainder of the code, as indicated by this copyright notice, is protected by copyright
and may not be used without authorization.

"""

import os
import shutil
import sys
import time
import numpy as np
import concurrent.futures
from weis.aeroelasticse.turbsim_util import Turbsim_wrapper

def run_turbsim(turbsim_input_file):
    wrapper = Turbsim_wrapper()
    wrapper.turbsim_exe = shutil.which('turbsim')
    wrapper.turbsim_input = turbsim_input_file
    wrapper.execute()

def run_TurbSim(wind_directory, number_of_workers):

    turbsim_input_files = [os.path.join(wind_directory, f) for f in os.listdir(wind_directory) if f.endswith('.inp')]

    with concurrent.futures.ProcessPoolExecutor(max_workers=number_of_workers) as executor:
        futures = []
        for turbsim_input_file in turbsim_input_files:
            # Check if the corresponding .bts file exists
            bts_file = os.path.splitext(turbsim_input_file)[0] + '.bts'
            if os.path.exists(bts_file):
                print(f"Skipping {turbsim_input_file} as {bts_file} already exists.")
                continue

            # If .bts file doesn't exist, add to the execution queue
            futures.append(executor.submit(run_turbsim, turbsim_input_file))

        for future in concurrent.futures.as_completed(futures):
            try:
                future.result()
            except Exception as e:
                print(f"Simulation failed with exception: {e}")
    
if __name__ == "__main__":
    wind_directory = sys.argv[1]
    number_of_workers = int(sys.argv[2])

    run_TurbSim(wind_directory, number_of_workers)
