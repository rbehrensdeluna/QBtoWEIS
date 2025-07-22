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
import subprocess

from pCrunch.io import OpenFASTOutput, OpenFASTBinary, OpenFASTAscii
from pCrunch import LoadsAnalysis, FatigueParams
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from packaging import version
import numpy as np  
import logging
import re
import sys
import time

weis_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))

magnitude_channels_default = {
    'LSShftF': ["X_s For. Shaft Const.", "Y_s For. Shaft Const.", "Z_s For. Shaft Const."], 
    'LSShftM': ["X_s Mom. Shaft Const.", "Y_s Mom. Shaft Const.", "Z_s Mom. Shaft Const."],
    'RootMc1': ["X_c RootBend. Mom. (IP) BLD_1", "Y_c RootBend. Mom. (OOP) BLD_1", "Z_c RootBend. Mom. BLD_1"],
    'RootMc2': ["X_c RootBend. Mom. (IP) BLD_2", "Y_c RootBend. Mom. (OOP) BLD_2", "Z_c RootBend. Mom. BLD_2"],
    'RootMc3': ["X_c RootBend. Mom. (IP) BLD_3", "Y_c RootBend. Mom. (OOP) BLD_3", "Z_c RootBend. Mom. BLD_3"],
    'TipDc1':  ['X_c Tip Trl.Def. (OOP) BLD_1', 'Y_c Tip Trl.Def. (IP) BLD_1', 'Z_c Tip Trl.Def. BLD_1'],
    'TipDc2':  ['X_c Tip Trl.Def. (OOP) BLD_2', 'Y_c Tip Trl.Def. (IP) BLD_2', 'Z_c Tip Trl.Def. BLD_2'],
    'TipDc3':  ['X_c Tip Trl.Def. (OOP) BLD_3', 'Y_c Tip Trl.Def. (IP) BLD_3', 'Z_c Tip Trl.Def. BLD_3'],
    'TwrBsM':  ['X_tb Mom. TWR Bot. Constr.', 'Y_tb Mom. TWR Bot. Constr.', 'Z_tb Mom. TWR Bot. Constr.'],
    'NcIMUTA': ['X_n Nac. Acc.','Y_n Nac. Acc.','Z_n Nac. Acc.']
}

fatigue_channels_default = {
    'RootMc1': FatigueParams(slope=10),
    'RootMc2': FatigueParams(slope=10),
    'RootMc3': FatigueParams(slope=10),
    'Y_b RootBend. Mom. BLD_1': FatigueParams(slope=10),   # 'RootMyb1': FatigueParams(slope=10),
    'Y_b RootBend. Mom. BLD_2': FatigueParams(slope=10),   # 'RootMyb2': FatigueParams(slope=10),
    'Y_b RootBend. Mom. BLD_3': FatigueParams(slope=10),   # 'RootMyb3': FatigueParams(slope=10),
    'TwrBsM': FatigueParams(slope=4),
    'LSShftM': FatigueParams(slope=4),
}
# all magnitude QBlade force and moment channells need to be scaled by 1e-3. These are the only ones that don't need to be scaled
no_scale_channels = {'TipDc1', 'TipDc2', 'TipDc3', 'NcIMUTA'}

logger = logging.getLogger("wisdem/weis") 

class QBladeWrapper:
    def __init__(self):
        self.QBlade_dll = None
        self.QBlade_libs = None
        self.QBLADE_runDirectory = None
        self.QBLADE_namingOut = None
        self.qb_vt = None
        self.qb_inumber = None

        self.channels           = {}
        self.number_of_workers  = 1
        self.cl_devices          = []
        self.cl_group_size      = 32
        self.no_structure       = False
        self.store_qprs         = False
        self.turbsim_params     = {}
        self.out_file_format    = 2
        self.delete_out_files   = True

        self.goodman            = False
        self.magnitude_channels = magnitude_channels_default
        self.fatigue_channels   = fatigue_channels_default
        self.la                 = None

    def init_crunch(self):
        if self.la is None:
            self.la = LoadsAnalysis(
                outputs=[],
                magnitude_channels=self.magnitude_channels,
                fatigue_channels=self.fatigue_channels,
                #extreme_channels=channel_extremes_default,
            )
    def run_qblade_cases(self):
        start_time = time.time()
        
        self.execute()

        end_time = time.time()
        elapsed_time = end_time - start_time
        print(f"Elapsed time to complete all QBlade simulation: {elapsed_time:.2f} seconds.")

        if self.number_of_workers == 1:
            summary_stats, extreme_table, DELs, Damage, ct =  self.run_serial()	
        else :
            summary_stats, extreme_table, DELs, Damage, ct =  self.run_multi()
        
        return summary_stats, extreme_table, DELs, Damage, ct
    
    def run_multi(self,): 
        self.init_crunch()

        # Filter only .out files and sort them
        all_files_in_dir = os.listdir(self.QBLADE_runDirectory)
        if self.out_file_format == 1:   # ASCII
            out_files = sorted([f for f in all_files_in_dir if f.endswith(".out")])
        elif self.out_file_format == 2: # Binary
            out_files = sorted([f for f in all_files_in_dir if f.endswith(".outb")])

        if sys.platform == "linux": # ProcessPoolExecutor is a bit quicker but doesn't work under windows
            with ProcessPoolExecutor(max_workers=self.number_of_workers) as executor:
                results = list(executor.map(self.parallel_analyze_cases, out_files))
        else:
            with ThreadPoolExecutor(max_workers=self.number_of_workers) as executor:
                results = list(executor.map(self.parallel_analyze_cases, out_files))

        ss = {}
        et = {}
        dl = {}
        dam = {}
        ct = []

        for (_name, _ss, _et, _dl, _dam, _ct) in results:
            ss[_name] = _ss
            et[_name] = _et
            dl[_name] = _dl
            dam[_name] = _dam
            ct.append(_ct)
            
        # Delete the .out files after processing
        if self.delete_out_files:
            for f in out_files:
                os.remove(os.path.join(self.QBLADE_runDirectory, f))
                print(f"Successfully deleted {f}.")

        summary_stats, extreme_table, DELs, Damage = self.la.post_process(ss, et, dl, dam)

        return summary_stats, extreme_table, DELs, Damage, ct

    def parallel_analyze_cases(self,file_name):
            QBLADE_Output_txt = os.path.join(self.QBLADE_runDirectory, file_name)
            return self.analyze_cases(QBLADE_Output_txt)
        
    def run_serial(self):
        self.init_crunch()

        # Filter only .out files and sort them
        all_files_in_dir = os.listdir(self.QBLADE_runDirectory)
        if self.out_file_format == 1:   # ASCII
            out_files = sorted([f for f in all_files_in_dir if f.endswith(".out")])
        elif self.out_file_format == 2: # Binary
            out_files = sorted([f for f in all_files_in_dir if f.endswith(".outb")])
        
        ss = {}
        et = {}
        dl = {}
        dam = {}
        ct = []

        for c in out_files:
            QBLADE_Output_txt = os.path.join(self.QBLADE_runDirectory, c)

            _name, _ss, _et, _dl, _dam, _ct = self.analyze_cases(QBLADE_Output_txt)
            ss[_name] = _ss
            et[_name] = _et
            dl[_name] = _dl
            dam[_name] = _dam
            ct.append(_ct)
        
        # Delete the .out files after processing
        if self.delete_out_files:
            for f in out_files:
                os.remove(os.path.join(self.QBLADE_runDirectory, f))
                print(f"Successfully deleted {f}.")
        
        summary_stats, extreme_table, DELs, Damage = self.la.post_process(ss, et, dl, dam)
        
        return summary_stats, extreme_table, DELs, Damage, ct
        
    def set_environment(self):
        # Set the environment variables to include the path to the shared libraries
        libraries_path = os.path.join(os.path.dirname(self.QBlade_dll), "Libraries")
        os.environ['LD_LIBRARY_PATH'] = f"{os.environ.get('LD_LIBRARY_PATH', '')}:{libraries_path}"
        print(f"Environment variables set. Library path: {libraries_path}")

    def execute(self):
        if sys.platform == "linux":
            self.set_environment()
        
        if sys.platform == 'win32':  
            dll_directory = os.path.dirname(self.QBlade_dll)
            os.environ["PATH"] = dll_directory + os.pathsep + os.environ.get("PATH", "")
            print(f"Added {dll_directory} to PATH")
            
        # Run the Python script using subprocess
        script_path = os.path.join(weis_dir, 'weis', 'aeroelasticse', 'QBlade_SIL.py')
        
        # Write the WEIS channels to a filter file to make QBlade only serilaize and export the channels we need
        filter_file = os.path.join(self.QBLADE_runDirectory,'QB_FILTERFILE.txt')
        with open(filter_file, 'w') as f:
            for channel in self.channels:
                f.write(channel + '\n')
        
        self.qblade_version_check()

        sim_params = [
            self.QBlade_dll, 
            self.QBLADE_runDirectory, 
            filter_file,
            str(self.number_of_workers),
            str(self.store_qprs),
            str(self.out_file_format),
            str(self.qb_inumber),
            str(self.cl_devices),
            str(self.cl_group_size),
            ]
        
        cmd = ['python', script_path] + sim_params
        subprocess.run(cmd, check=True)


    def analyze_cases(self, case):
        if self.out_file_format == 1 and os.path.exists(case):
            output_init = OpenFASTAscii(case, magnitude_channels=self.magnitude_channels)
        if self.out_file_format == 2 and  os.path.exists(case):
            output_init = OpenFASTBinary(case, magnitude_channels=self.magnitude_channels)

        output_init.read()

        # Make output dict
        output_dict = {}
        filename = os.path.basename(case)

        # Iterate through channels and apply scaling
        for idx, channel in enumerate(output_init.channels):
            # Determine the unit for the channel
            if idx < len(output_init.units):
                unit = output_init.units[idx]
            else:
                unit = None  # If no unit is provided

            if unit:
                # If there is a unit, scale according to the unit logic
                scaled_data, scaled_unit = self.scale_channels(output_init.df[channel].to_numpy(), unit)
            else:
                # If there is no unit, check if the channel is in the no_scale_channels list
                if channel in no_scale_channels:
                    # If it's in the no_scale_channels list, don't scale
                    scaled_data = output_init.df[channel].to_numpy()
                    scaled_unit = unit
                else:
                    # If it's not in the list, scale by 1e-03
                    scaled_data = output_init.df[channel].to_numpy() * 1e-03
                    scaled_unit = unit

            # Store the scaled data
            output_dict[channel] = scaled_data

        # Re-make output
        output = OpenFASTOutput.from_dict(output_dict, filename)

        # Trim Data
        if self.qb_vt['QSim']['STOREFROM'] > 0.0 and not self.qb_vt['QSim']['DLCGenerator']: # in DLCGenerator QBlade never stores the values during the "tansient_time"
            output.trim_data(tmin=self.qb_vt['QSim']['STOREFROM'], tmax=self.qb_vt['QSim']['TMax'])
        case_name, sum_stats, extremes, dels, damage = self.la._process_output(output,
                                                                            return_damage=True,
                                                                            goodman_correction=self.goodman)
        
        return case_name, sum_stats, extremes, dels, damage, output_dict        

    def qblade_version_check(self):
        match = re.search(r'(\d+\.\d+\.\d+(\.\d+)?)', self.QBlade_dll) # Extract the version from self.QBlade_dll
        if match:
            qb_version = match.group(1)
        else:
            raise ValueError("Version number not found in QBlade_dll path.")
        
        mp_version = "2.0.8.7" # version that exports to outb format directly

        if version.parse(qb_version) < version.parse(mp_version):
            print("Error: QBlade version:", version.parse(qb_version), "not compatible with QBtoWEIS. Please use QBlade Version 2.0.8.7 or newer.")
            sys.exit(1)
        else:
            print("QBlade version: ", version.parse(qb_version), "was found!")
    
    def scale_channels(self, data, unit):
        """
        Scale [N] -> [kN], [Nm] -> [kNm], and [W] -> [kW]
        """
        scaled_data = data  # default to unscaled
        scaled_unit = unit  # default to original unit

        if unit.endswith("N"):
            scaled_data = data * 1e-03
            scaled_unit = "kN"
        elif unit.endswith("Nm"):
            scaled_data = data * 1e-03
            scaled_unit = "kNm"
        elif unit.endswith("W"):
            scaled_data = data * 1e-03
            scaled_unit = "kW"

        return scaled_data, scaled_unit


# for testing
if __name__ == "__main__":
    dll_path = "/home/robert/qblade/software/QBladeCE_2.0.8.5/libQBladeCE_2.0.8.5.so.1.0.0"
    libs_path = "/home/robert/qblade/software/QBladeCE_2.0.8.5/Libraries"
    run_directory = "/home/robert/floatfarm/QBtoWEIS/qb_examples/MED15-300_v09.5.1_opt/m_output_MED15-300_v09.6.2/"
    naming_out = "MED15-300_v09.6.2"
    
    qblade = QBladeWrapper()
    qblade.QBlade_dll = dll_path
    qblade.QBlade_libs = libs_path
    qblade.QBLADE_runDirectory = run_directory
    qblade.QBLADE_namingOut    = naming_out
    qb_vt = {
        'QSim': {
            'NUMTIMESTEPS': 3000,
            'TMax': 0,
            'STOREFROM': 0,
        },
    }
    qblade.qb_vt = qb_vt
    qblade.number_of_workers    = 15
    qblade.no_structure         = False
    qblade.store_qprs           = False
    qblade.chunk_size           = 30000000
    qblade.out_file_format      = 2
    qblade.delete_out_files     = False
    qblade.cl_device            = [1]
    qblade.cl_group_size        = 32
    # failed = qblade.execute()
    summary_stats, extreme_table, DELs, Damage, ct =  qblade.run_multi()