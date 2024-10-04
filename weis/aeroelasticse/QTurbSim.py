
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
import time
import numpy as np
import concurrent.futures
from weis.aeroelasticse.turbsim_util import Turbsim_wrapper

class TurbSimRunner:
    def __init__(self):
        self.QBLADE_runDirectory = None
        self.QBLADE_namingOut = None
        self.qb_vt = None
        self.number_of_workers = 1

    def run_single_turbsim(self, idx):
        QBLADE_namingOut_appendix = f'_{idx}'
        wind_directory = os.path.join(self.QBLADE_runDirectory, self.QBLADE_namingOut + QBLADE_namingOut_appendix, 'wind')
        wrapper = Turbsim_wrapper()
        wrapper.run_dir = wind_directory
        wrapper.turbsim_exe = shutil.which('turbsim')
        wrapper.turbsim_input = self.QBLADE_namingOut + QBLADE_namingOut_appendix + '.inp'
        wrapper.execute()

    def run_TurbSim(self):
        indices = np.arange(len(self.qb_vt['QTurbSim']['URef']))
        
        with concurrent.futures.ProcessPoolExecutor(max_workers=self.number_of_workers) as executor:
            futures = [executor.submit(self.run_single_turbsim, idx) for idx in indices]
            
            for future in concurrent.futures.as_completed(futures):
                try:
                    future.result()
                except Exception as e:
                    print(f"Simulation failed with exception: {e}")
