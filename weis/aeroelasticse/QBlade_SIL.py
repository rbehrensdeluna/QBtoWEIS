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

from ctypes import *
from QBladeLibrary import QBladeLibrary
import sys
import concurrent.futures
import shutil
import stat
import time
import os
import numpy as np
import pandas as pd


def qblade_sil(QBlade_dll, QBLADE_runDirectory, sim, n_dt, channels, no_structure, store_qprs, store_from):
    bsim = sim.encode("utf-8")
    sim_name = os.path.basename(sim)
    # We don't need this at the moment but it will live here in case we do:
    # dll_directory = dll_directory = os.path.dirname(QBlade_dll)

    # if sys.platform == 'win32':  # 'nt' indicates Windows
    #     os.environ["PATH"] = os.path.abspath(dll_directory) + ";" + os.environ.get("PATH", "")
    
    QBLIB = QBladeLibrary(QBlade_dll)
    QBLIB.createInstance(1,32) 
    QBLIB.setOmpNumThreads(1)
    QBLIB.loadSimDefinition(bsim)
    QBLIB.initializeSimulation()
    QBLIB.setAutoClearTemp(False)
    
    # Convert each item in the list to a bytes-like object
    channels += ['qblade_failed [-]']
    bchannels = [bytes(channel, 'utf-8') for channel in channels]
    output_dict = {channel: [] for channel in channels}

    simulation_completed = True
    start_time = time.time()
    for i in range(n_dt):

        success = QBLIB.advanceTurbineSimulation() 

        # Check if the simulation step was successful
        if not success:
            print(f"Simulation {sim_name} failed at timestep {i}, exiting simulation loop", flush=True)
            simulation_completed = False
            output_dict['qblade_failed [-]'] = np.ones_like(output_dict['Time [s]'])
            break

        if 'False' in no_structure: # we can only advance the controller as long as a structural model is included in the simulation
            ctr_vars = (c_double * 5)(0) 
            QBLIB.advanceController_at_num(ctr_vars,0) 
            # QBLIB.setControlVars_at_num(ctr_vars,0) 

        if i % (n_dt // 10) == 0:
            progress_percentage = i / n_dt * 100
            elapsed_time = time.time() - start_time
            print(f"Simulation Progress: {sim_name} at {progress_percentage:3.0f}% (time elapsed: {elapsed_time:.1f} s)", flush=True)


        # extract channels from simulation    
        if QBLIB.getCustomData_at_num(b'Time [s]', 0, 0) >= store_from:
            for bchannel, channel in zip(bchannels, channels):
                    data = QBLIB.getCustomData_at_num(bchannel, 0, 0)
                    output_dict[channel].append(data)

    if simulation_completed:
        print(f"Simulation Progress: {sim_name} complete (time elapsed: {elapsed_time:.1f} s)", flush=True)
        output_dict['qblade_failed [-]'] = np.zeros_like(output_dict['Time [s]'])

    sim_out_name = sim.strip('.sim')
    
    if 'True' in store_qprs:
        output_file = f"{sim_out_name}_completed.qpr".encode('ASCII')
        QBLIB.storeProject(output_file)
    
    QBLIB.closeInstance()
    del QBLIB.lib

    # Scale and rename channels and write them to an OF typ ASCII file
    output_dict = scale_and_rename_channels(output_dict)
    export_to_OF_ASCII(output_dict, directory = QBLADE_runDirectory,  filename = sim_out_name + '_completed.out')

def run_with_retry(QBlade_dll, QBLADE_runDirectory, sim, n_dt, channels, no_structure, store_qprs, store_from, max_retries=2, delay=2):
    attempt = 0
    while attempt < max_retries:
        try:
            print(f"Running simulation attempt {attempt + 1} for {sim}...")
            qblade_sil(QBlade_dll, QBLADE_runDirectory, sim, n_dt, channels, no_structure, store_qprs, store_from)
            return  # Exit if successful
        except Exception as e:
            print(f"Simulation attempt {attempt + 1} failed with exception: {e}")
            attempt += 1
            if attempt < max_retries:
                print("Retrying...")
                time.sleep(delay)
            else:
                print(f"Max retries reached for {sim}. Moving on.")

def run_with_retry(QBlade_dll, QBLADE_runDirectory, sim, n_dt, channels, no_structure, store_qprs, store_from, max_retries=2, delay=2):
    attempt = 0
    while attempt < max_retries:
        try:
            print(f"Running simulation attempt {attempt + 1} for {sim}...")
            qblade_sil(QBlade_dll, QBLADE_runDirectory, sim, n_dt, channels, no_structure, store_qprs, store_from)
            return  # Exit if successful
        except Exception as e:
            print(f"Simulation attempt {attempt + 1} failed with exception: {e}")
            attempt += 1
            if attempt < max_retries:
                print("Retrying...")
                time.sleep(delay)
            else:
                print(f"Max retries reached for {sim}. Moving on.")

def run_qblade_sil(QBlade_dll, QBLADE_runDirectory, channels, n_dt, number_of_workers, no_structure, store_qprs, store_from):
    
    clear_and_delete_temp(QBlade_dll) # delete TEMP folder within QBlade directory to prevent unnecessary data clogging

    simulations = [os.path.join(QBLADE_runDirectory, f) for f in os.listdir(QBLADE_runDirectory) if f.endswith('.sim')]

    with concurrent.futures.ProcessPoolExecutor(max_workers=number_of_workers) as executor:
        futures = []
        for sim in simulations:
            time.sleep(1)  # Introduce a one-second pause before submitting the next task
            futures.append(
                executor.submit(run_with_retry, QBlade_dll, QBLADE_runDirectory, sim, n_dt, channels, no_structure, store_qprs, store_from)
            )
        for future in concurrent.futures.as_completed(futures):
            try:
                future.result()
            except Exception as e:
                print(f"Simulation failed after retrying with exception: {e}")
    
def export_to_OF_ASCII(data, directory=None, filename=None):
    """
    Writes a results file in OpenFAST ASCII format. 
    """ 
    # split channel name and unit
    channels = []
    units = []
    for idx, entry in enumerate(data):
        split_string = entry.split(" [")
        channels.append(split_string[0])
        # channels.append(split_string[0].replace(' ','_')) # This would be required to make the result file compatible with PDAP
        units.append(''.join(('(',split_string[1].strip("]"),')')))
    dataframe = pd.DataFrame(data)

    # create dataframe with 2 headers (1. channel names, 2. units)
    col_names = dict(zip(dataframe.columns, channels))
    dataframe = dataframe.rename(col_names, axis=1)
    units_df = pd.DataFrame([units], columns=channels)
    dataframe = pd.concat([units_df, dataframe], ignore_index=True) 

    # write to file
    if directory is not None:
        if not os.path.exists(directory):
            os.makedirs(directory)
        file_path = os.path.join(directory, filename)

    # First, write the additional lines to the file
    with open(file_path, 'w') as f:
        f.write('---------------------------------------- file generated with WEIS QBlade API ----------------------------------------\n')
        f.write('Results are written in OpenFAST ASCII (.out) format\n')
        f.write(filename)
        f.write("\n \n \n \n")

    # Convert DataFrame to a tab-separated string and write to .out file
    data_string = dataframe.to_csv(sep='\t', index=False, header=True)
    with open(file_path, 'a') as f:
        f.write(data_string)
    
    print(f"Simulation result {directory} successfully exported")

def scale_and_rename_channels(output_dict):
    """
    Scale [N] -> [kN] and [Nm] -> [kNm]
    """
    for channel in list(output_dict.keys()):
        if "[N]" in channel:
            output_dict[channel] = [value * 1e-03 for value in output_dict[channel]]
            new_channel_name = channel.replace("[N]", "[kN]")
            output_dict[new_channel_name] = output_dict.pop(channel)
            
        elif "[Nm]" in channel:
            output_dict[channel] = [value * 1e-03 for value in output_dict[channel]]
            new_channel_name = channel.replace("[Nm]", "[kNm]")
            output_dict[new_channel_name] = output_dict.pop(channel)
        
        elif "[W]" in channel:
            output_dict[channel] = [value * 1e-03 for value in output_dict[channel]]
            new_channel_name = channel.replace("[W]", "[kW]")
            output_dict[new_channel_name] = output_dict.pop(channel)
        
    return output_dict

def handle_remove_error(func, path, exc_info):
    """Handles file/folder removal errors by adjusting permissions and retrying."""
    try:
        # Remove read-only flag (Windows) or add write permission (Linux)
        if os.name == "nt":  # Windows
            os.chmod(path, stat.S_IWRITE)
        else:  # Linux/macOS
            os.chmod(path, stat.S_IWUSR)  # User write permission

        func(path)  # Retry deletion
    except Exception as e:
        print(f"Failed to remove {path}: {e}")

def clear_and_delete_temp(qblade_dll):
    """Removes the read-only attribute, clears TEMP folder contents, and deletes it."""
    qblade_dir = os.path.dirname(qblade_dll)
    temp_path = os.path.join(qblade_dir, "TEMP")

    if os.path.exists(temp_path) and os.path.isdir(temp_path):
        try:
            # Remove all files and subdirectories first
            for root, dirs, files in os.walk(temp_path, topdown=False):
                for file in files:
                    file_path = os.path.join(root, file)
                    try:
                        os.chmod(file_path, stat.S_IWRITE)  # Remove read-only attribute
                        os.remove(file_path)  # Delete file
                    except Exception as e:
                        print(f"Failed to delete file {file_path}: {e}")
                        
                for dir in dirs:
                    dir_path = os.path.join(root, dir)
                    try:
                        os.chmod(dir_path, stat.S_IWRITE)  # Remove read-only attribute
                        os.rmdir(dir_path)  # Remove empty directory
                    except Exception as e:
                        print(f"Failed to delete directory {dir_path}: {e}")

            # Now delete the TEMP folder itself, using onexc to handle errors
            shutil.rmtree(temp_path, onexc=handle_remove_error)
            print("TEMP folder successfully deleted.")
        except Exception as e:
            print(f"Failed to delete TEMP folder: {e}")
        
if __name__ == "__main__":
    
    QBlade_dll = sys.argv[1]
    QBLADE_runDirectory = sys.argv[2]
    channels_str = sys.argv[3]
    n_dt = int(sys.argv[4])
    number_of_workers = int(sys.argv[5])
    no_structure = sys.argv[6]
    store_qprs = sys.argv[7]
    store_from = float(sys.argv[8])

    # required inputs are converted back into the datatype that we need
    channels = channels_str.split(',') #convert back to list

    run_qblade_sil(QBlade_dll, QBLADE_runDirectory, channels, n_dt, number_of_workers, no_structure, store_qprs, store_from)
