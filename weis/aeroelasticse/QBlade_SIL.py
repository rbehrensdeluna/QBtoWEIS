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
import struct as st
import yaml


max_retries = 5 # Number of retries for creating an instance in case license is not validaded by the server

def qblade_sil(QBlade_dll, QBLADE_runDirectory, sim, channels, store_qprs, out_file_format, qb_inumber, cl_device, cl_group_size):
    bsim = sim.encode("utf-8")
    sim_name = os.path.basename(sim)

    QBLIB = QBladeLibrary(QBlade_dll)

    for attempt in range(max_retries):
        if QBLIB.createInstance(cl_device, cl_group_size):
            success = True
            print(f"Instance created successfully for {sim} on attempt {attempt + 1}.")
            break
        else:
            print(f"Attempt {attempt + 1} failed for {sim}.")
            time.sleep(1)
    if not success:
        raise RuntimeError("Failed to create instance after 5 attempts.")
    
    QBLIB.setOmpNumThreads(1)
    QBLIB.loadSimDefinition(bsim)
    QBLIB.initializeSimulation()
    QBLIB.setAutoCleanup(False)

    simulation_success = QBLIB.runFullSimulation()
    
    if not simulation_success:
        log_failed_simulation(sim_name, qb_inumber, QBLADE_runDirectory)
        raise RuntimeError(f"Simulation {sim} failed.")  
    
    sim_out_name = sim_name.strip('.sim')
    
    # TODO: allow for out AND oub
    if out_file_format == 2 and simulation_success: # 2 --> binary:
        # QBLIB.exportResults(3, QBLADE_runDirectory.encode(), (sim_out_name + '_completed').encode(), channels.encode()) # this is required to get the time channel
        QBLIB.exportResults(3, QBLADE_runDirectory.encode(), (sim_out_name + '_completed').encode(), ''.encode())
    else:
        raise ValueError("Error: Only 'outb' format is supported for binary export (out_file_format = 2). 'out' is no longer supported.")
        
    if 'True' in store_qprs:
        qpr_project = os.path.join(QBLADE_runDirectory, f"{sim_out_name}_completed.qpr".encode('ASCII'))
        QBLIB.storeProject(qpr_project)
    
    QBLIB.unload()

def run_qblade_sil(QBlade_dll, QBLADE_runDirectory, channels, number_of_workers, store_qprs, out_file_format, qb_inumber, cl_devices, cl_group_size):
    
    clear_and_delete_temp(QBlade_dll) # delete TEMP folder within QBlade directory to prevent unnecessary data clogging

    simulations = sorted([os.path.join(QBLADE_runDirectory, f) for f in os.listdir(QBLADE_runDirectory) if f.endswith('.sim')])
    num_cl_devices = len(cl_devices)

    # Chunk the simulations for each device
    sim_chunks = np.array_split(simulations, num_cl_devices)
    
    # Useful if some simulations take a lot longer than others
    # sim_chunks = [simulations[i::num_cl_devices] for i in range(num_cl_devices)]

    with concurrent.futures.ProcessPoolExecutor(max_workers=number_of_workers) as executor:
        futures = []

        # Distribute simulations evenly across cl_devices        
        for device_index, cl_device in enumerate(cl_devices):
            for sim in sim_chunks[device_index]:
                futures.append(
                    executor.submit(
                        qblade_sil,
                        QBlade_dll,
                        QBLADE_runDirectory,
                        sim,
                        channels,
                        store_qprs,
                        out_file_format,
                        qb_inumber,
                        cl_device,           # Correct cl_device assigned to this chunk
                        cl_group_size
                    )
                )
                time.sleep(0.25)  # Optional: prevent overloading
        for future in concurrent.futures.as_completed(futures):
            try:
                future.result()
            except Exception as e:
                print(f"Simulation failed with exception: {e}")

def log_failed_simulation(sim_name, qb_inumber, run_directory):
    status_file = os.path.join(run_directory, "qblade_run_failure_log.yaml")
    key = f"iteration_{qb_inumber:03d}"

    # Load existing failure log
    if os.path.exists(status_file):
        with open(status_file, "r") as f:
            try:
                failures = yaml.safe_load(f) or {}
            except yaml.YAMLError:
                failures = {}
    else:
        failures = {}

    # Initialize structure if necessary
    if key not in failures:
        failures[key] = {"failed_simulations": []}

    # Append simulation name if not already listed
    if sim_name not in failures[key]["failed_simulations"]:
        failures[key]["failed_simulations"].append(sim_name)

    # Write updated log
    with open(status_file, "w") as f:
        yaml.dump(failures, f)

def export_to_OF_Binary(data, outfilename):
    """
    Writes a results file in OpenFAST Binary format.
    
    Parameters:
    - data: A DataFrame with channel names and data. Channel names are in the format 'ChannelName [Unit]'.
    - outfilename: The path to the output binary file.
    """

    FileID = 4
    DescStr = "Generated by export_to_OF_Binary"
    PackingData = None

    channels = []
    ChanUnit = []

    for idx, entry in enumerate(data):
        split_string = entry.split(" [")
        channels.append(split_string[0])
        # channels.append(split_string[0].replace(' ','_'))
        ChanUnit.append(''.join(('(',split_string[1].strip("]"),')')))

    dataframe = pd.DataFrame(data)
    col_names = dict(zip(dataframe.columns, channels))
    Chans = dataframe.rename(col_names, axis=1)

    if 'Time' not in Chans.columns:
        raise ValueError("'Time' column is required and must be named exactly 'Time'")

    cols = Chans.columns.tolist()
    cols.insert(0, cols.pop(cols.index('Time')))  # Move 'Time' to front
    Chans = Chans[cols]
    
    # Scaling Parameters                                 
    IntMax = np.float64(32757.0)                            
    IntMin = np.float64(-32768.0)                           
    IntRang = np.float64(IntMax-IntMin)                     

    # Time parameters
    Time = np.float64(Chans.Time)                                           
    TimeOut1 = Time[0]                                      
    TimeIncrement = Time[1]-Time[0]                         

    # The Channels without time
    ChansMod = Chans.drop("Time", axis=1)
    ColMax = np.float64(ChansMod.max())                     
    ColMin = np.float64(ChansMod.min())                     
    ColOff = []                                             
    ColScl = []                                             

    LenDesc = len(DescStr)                                  
    NT = len(Time)                                          
    NumOutChans = len(ChansMod.columns)                                        

    TmpOutArray = np.zeros((ChansMod.size), dtype=np.int16)                                            
    
    maxChanLen = max(len(name) for name in channels)
    maxUnitLen = max(len(unit) for unit in ChanUnit)
    nChar = max(maxChanLen, maxUnitLen)        

    if PackingData is None:
        for i in range(len(ColMax)):
            if ColMax[i] == ColMin[i]:
                ColScl.append(np.float32(1))
            else:
                ColScl.append(np.float32(IntRang / (ColMax[i] - ColMin[i])))
            ColOff.append(np.float32(IntMin - ColScl[i] * ColMin[i]))
    else:
        ColScl = PackingData['ColScl']
        ColOff = PackingData['ColOff']                         

    TempFrame = ChansMod.copy()
    for j in range(NumOutChans):
        TempFrame[TempFrame.columns[j]] = ColScl[j] * TempFrame[TempFrame.columns[j]] + ColOff[j]
    TmpOutArray = np.clip(TempFrame.values.flatten(), IntMin, IntMax).astype(np.int16)

    ChanNameASCII = [name[:nChar].ljust(nChar).encode('latin_1') for name in Chans.columns]
    ChanUnitASCII = [unit[:nChar].ljust(nChar).encode('latin_1') for unit in ChanUnit]
    
    with open(outfilename, 'wb') as outfile:
        outfile.write(st.pack('h', np.int16(FileID)))
        outfile.write(st.pack('@h', np.int16(nChar)))
        outfile.write(st.pack('i', np.int32(NumOutChans)))
        outfile.write(st.pack('i', np.int32(NT)))

        outfile.write(st.pack('d', np.float64(TimeOut1)))
        outfile.write(st.pack('d', np.float64(TimeIncrement)))

        outfile.write(np.array(ColScl, dtype=np.float32))
        outfile.write(np.array(ColOff, dtype=np.float32))

        outfile.write(np.int32(LenDesc))
        outfile.write(DescStr.encode('utf-8'))

        

        outfile.write(np.array(ChanNameASCII))
        outfile.write(np.array(ChanUnitASCII))

        outfile.write(TmpOutArray)

def export_to_OF_ASCII(data, directory=None, filename=None, first_chunk=True):
    """
    Writes a results file in OpenFAST ASCII format. 
    """ 
    # split channel name and unit
    channels = []
    units = []
    for idx, entry in enumerate(data):
        split_string = entry.split(" [")
        channels.append(split_string[0])
        # channels.append(split_string[0].replace(' ','_')) # This would be required to make the result file compatible with PDAP becaus blank spaces are not allowed
        units.append(''.join(('(',split_string[1].strip("]"),')')))
    dataframe = pd.DataFrame(data)

    # create dataframe with 2 headers (1. channel names, 2. units)
    dataframe = pd.DataFrame(data)
    col_names = dict(zip(dataframe.columns, channels))
    dataframe = dataframe.rename(col_names, axis=1)
    units_df = pd.DataFrame([units], columns=channels)
    dataframe = pd.concat([units_df, dataframe], ignore_index=True)

    # Apply scientific notation format with 6 decimal places
    dataframe = dataframe.apply(lambda col: col.map(lambda x: f"{x:.6E}" if isinstance(x, (int, float)) else x))

    if first_chunk:  # Write header only for the first chunk of data
        with open(filename, 'w') as f:  # Open in 'w' mode for the first chunk
            f.write('---------------------------------------- file generated with WEIS QBlade API ----------------------------------------\n')
            f.write('Results are written in OpenFAST ASCII (.out) format\n')
            f.write(os.path.basename(filename)) # write the filename
            f.write("\n \n \n \n")
            data_string = dataframe.to_csv(sep='\t', index=False, header=True, lineterminator='\n')
            f.write(data_string)

    else:  # Append data for subsequent chunks,
        with open(filename, 'a') as f:  # Open in 'a' mode to append
            data_string = dataframe.iloc[1:].to_csv(sep='\t', index=False, header=False, lineterminator='\n')  # Skip first two rows
            f.write(data_string)

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
    channels = sys.argv[3]
    number_of_workers = int(sys.argv[4])
    store_qprs = sys.argv[5]
    out_file_format =  float(sys.argv[6])
    qb_inumber = int(sys.argv[7])
    cl_devices =  sys.argv[8]
    cl_group_size = int(sys.argv[9])

    # convert string back to array:
    cl_devices = np.array(sys.argv[8][1:-1].split(','), dtype=int)

    run_qblade_sil(QBlade_dll, QBLADE_runDirectory, channels, number_of_workers, store_qprs, out_file_format, qb_inumber, cl_devices, cl_group_size)
