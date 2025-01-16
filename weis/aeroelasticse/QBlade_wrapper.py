
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
from packaging import version
import logging
import re
import sys

weis_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))

magnitude_channels_default = {
    'LSShftF': ["X_s For. Shaft Const.", "Y_s For. Shaft Const.", "Z_s For. Shaft Const."], 
    'LSShftM': ["X_s Mom. Shaft Const.", "Y_s Mom. Shaft Const.", "Z_s Mom. Shaft Const."],
    'RootMc1': ["X_c RootBend. Mom. (IP) BLD 1", "Y_c RootBend. Mom. (OOP) BLD 1", "Z_c RootBend. Mom. BLD 1"],
    'RootMc2': ["X_c RootBend. Mom. (IP) BLD 2", "Y_c RootBend. Mom. (OOP) BLD 2", "Z_c RootBend. Mom. BLD 2"],
    'RootMc3': ["X_c RootBend. Mom. (IP) BLD 3", "Y_c RootBend. Mom. (OOP) BLD 3", "Z_c RootBend. Mom. BLD 3"],
    'TipDc1':  ['X_c Tip Trl.Def. (OOP) BLD 1', 'Y_c Tip Trl.Def. (IP) BLD 1', 'Z_c Tip Trl.Def. BLD 1'],
    'TipDc2':  ['X_c Tip Trl.Def. (OOP) BLD 2', 'Y_c Tip Trl.Def. (IP) BLD 2', 'Z_c Tip Trl.Def. BLD 2'],
    'TipDc3':  ['X_c Tip Trl.Def. (OOP) BLD 3', 'Y_c Tip Trl.Def. (IP) BLD 3', 'Z_c Tip Trl.Def. BLD 3'],
    'TwrBsM':  ['X_tb Mom. TWR Bot. Constr.', 'Y_tb Mom. TWR Bot. Constr.', 'Z_tb Mom. TWR Bot. Constr.'],
    'NcIMUTA': ['X_n Nac. Acc.','Y_n Nac. Acc.','Z_n Nac. Acc.']
}

fatigue_channels_default = {
    'RootMc1': FatigueParams(slope=10),
    'RootMc2': FatigueParams(slope=10),
    'RootMc3': FatigueParams(slope=10),
    'Y_b RootBend. Mom. BLD 1': FatigueParams(slope=10),   # 'RootMyb1': FatigueParams(slope=10),
    'Y_b RootBend. Mom. BLD 2': FatigueParams(slope=10),   # 'RootMyb2': FatigueParams(slope=10),
    'Y_b RootBend. Mom. BLD 3': FatigueParams(slope=10),   # 'RootMyb3': FatigueParams(slope=10),
    'TwrBsM': FatigueParams(slope=4),
    'LSShftM': FatigueParams(slope=4),
}

logger = logging.getLogger("wisdem/weis") 

class QBladeWrapper:
    def __init__(self):
        self.QBlade_dll = None
        self.QBlade_libs = None
        self.QBLADE_runDirectory = None
        self.QBLADE_namingOut = None
        self.qb_vt = None

        self.channels           = {}
        self.number_of_workers  = 1
        self.no_structure       = False
        self.store_qprs         = False
        self.turbsim_params     = {}

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
    def run_qblade_cases(self): #equivalent to run_serial/run_multi 
        
        self.execute()
        self.init_crunch()
        
        ss = {}
        et = {}
        dl = {}
        dam = {}
        ct = []
        
        for c in os.listdir(self.QBLADE_runDirectory):
            if c.endswith(".out"):
                QBLADE_Output_txt = os.path.join(self.QBLADE_runDirectory, c)

                _name, _ss, _et, _dl, _dam, _ct = self.analyze_cases(QBLADE_Output_txt)
                ss[_name] = _ss
                et[_name] = _et
                dl[_name] = _dl
                dam[_name] = _dam
                ct.append(_ct)
        
        summary_stats, extreme_table, DELs, Damage = self.la.post_process(ss, et, dl, dam)
        
        return summary_stats, extreme_table, DELs, Damage, ct
        
    def set_environment(self):
        # Set the environment variables to include the path to the shared libraries
        libraries_path = self.QBlade_libs
        os.environ['LD_LIBRARY_PATH'] = f"{os.environ.get('LD_LIBRARY_PATH', '')}:{libraries_path}"
        print(f"Environment variables set. Library path: {libraries_path}")

    def execute(self):
        if sys.platform == "linux":
            self.set_environment()
        # Run the Python script using subprocess
        script_path = os.path.join(weis_dir, 'weis', 'aeroelasticse', 'QBlade_SIL.py')

        channels_str = ','.join(self.channels)  # convert channels to csv

        if self.qb_vt['QSim']['TMax'] > 0:
            self.qb_vt['QSim']['NUMTIMESTEPS'] = int(self.qb_vt['QSim']['TMax'] / self.qb_vt['QSim']['TIMESTEP'])
        
        QB_mp_compatible = self.qblade_version_check()

        sim_params = [
            self.QBlade_dll, 
            self.QBLADE_runDirectory, 
            channels_str, 
            str(self.qb_vt['QSim']['NUMTIMESTEPS']),
            str(self.number_of_workers),
            str(self.no_structure),
            str(self.store_qprs),
            str(QB_mp_compatible),
            str(self.qb_vt['QSim']['STOREFROM']),
            ]
        
        cmd = ['python', script_path] + sim_params
        subprocess.run(cmd, check=True)


    def analyze_cases(self, case):
        if os.path.exists(case):
            output_init = OpenFASTAscii(case, magnitude_channels=self.magnitude_channels)

        output_init.read()

        # Make output dict
        output_dict = {}
        filename = os.path.basename(case)
        for channel in output_init.channels:
            output_dict[channel] = output_init.df[channel].to_numpy()

        # Re-make output
        output = OpenFASTOutput.from_dict(output_dict, filename)

        # Trim Data
        if self.qb_vt['QSim']['STOREFROM'] > 0.0:
            output.trim_data(tmin=self.qb_vt['QSim']['STOREFROM'], tmax=self.qb_vt['QSim']['TMax'])
        case_name, sum_stats, extremes, dels, damage = self.la._process_output(output,
                                                                            return_damage=True,
                                                                            goodman_correction=self.goodman)
        
        return case_name, sum_stats, extremes, dels, damage, output_dict        

    def qblade_version_check(self):
        match = re.search(r'QBladeCE_(\d+\.\d+\.\d+(\.\d+)?)', self.QBlade_dll) # Extract the version from self.QBlade_dll
        if match:
            qb_version = match.group(1)
        else:
            raise ValueError("Version number not found in QBlade_dll path.")
        
        mp_version = "2.0.7.8" # version that includes mp capability and allows for number_of_workers > 1
            
        if version.parse(qb_version) < version.parse(mp_version):
            self.number_of_workers = 1
            QB_mp_compatible = False
            logger.warning("WARNING: This QBlade version is older than v2.0.7.8 and does not support mp, hence the number of workers is forced to be 1!!")
        else:
            QB_mp_compatible = True

        return QB_mp_compatible
# for testing
if __name__ == "__main__":
    dll_path = "/home/robert/GitHub/QBtoWEIS-Coupling/QBlade/QBladeCE_2.0.7_linux/QBladeCE_2.0.7/libQBladeCE_2.0.7.so.1.0.0"
    libs_path = "/home/robert/GitHub/QBtoWEIS-Coupling/QBlade/QBladeCE_2.0.7_linux/QBladeCE_2.0.7/Libraries"
    run_directory = "/home/robert/GitHub/QBtoWEIS-Coupling/testing/QBlade_wrapper/outputs/qb_test_nrel5mw"
    naming_out = "qb_test_nrel5mw"
    
    qblade = QBladeWrapper()
    qblade.QBlade_dll = dll_path
    qblade.QBlade_libs = libs_path
    qblade.QBLADE_runDirectory = run_directory
    qblade.QBLADE_namingOut    = naming_out
    qb_vt = {
        'QSim': {
            'NUMTIMESTEPS': 4000
        }
    }
    qblade.qb_vt = qb_vt

    failed = qblade.execute()
