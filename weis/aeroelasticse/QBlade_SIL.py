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
import gc
import struct as st

def qblade_sil(QBlade_dll, QBLADE_runDirectory, sim, n_dt, channels, no_structure, store_qprs, store_from, chunk_size, out_file_format):
    bsim = sim.encode("utf-8")
    sim_name = os.path.basename(sim)
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
    sim_out_name = sim.strip('.sim')
    file_path = os.path.join(QBLADE_runDirectory, sim_out_name + '_completed.out')

    simulation_completed = True
    start_time = time.time()
    first_chunk = True 
    for i in range(n_dt):

        success = QBLIB.advanceTurbineSimulation() 

        # Check if the simulation step was successful
        if not success:
            print(f"Simulation {sim_name} failed at timestep {i}, exiting simulation loop", flush=True)
            simulation_completed = False
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
                if isinstance(data, (float, int)):
                    rounded_data = round(data, 5)
                    output_dict[channel].append(rounded_data)
                else:
                    output_dict[channel].append(data)
        
        if (i + 1) % chunk_size == 0 and out_file_format == 1:  # Write every chunk_size, only availble for ascii type files
            print(f"Writing chunk to {file_path}...", flush=True)
            output_dict['qblade_failed [-]'] = np.zeros_like(output_dict['Time [s]'])
            output_dict = scale_and_rename_channels(output_dict)
            export_to_OF_ASCII(output_dict, directory=None, filename=file_path, first_chunk=first_chunk)
            print(f"Chunk successfully written to {file_path}, continuing simulation...", flush=True)

            # Reset dictionary and prepare for next chunk
            output_dict.clear()
            gc.collect()  # Force garbage collection to free up memory
            output_dict = {channel: [] for channel in channels}
            first_chunk = False # set to False after the first chunk has been written so the header is not written again
        
    if simulation_completed:  # This now handles ALL remaining data
        print(f"Simulation Progress: {sim_name} complete (time elapsed: {elapsed_time:.1f} s); finalizing simulation data...", flush=True)
        output_dict['qblade_failed [-]'] = np.zeros_like(output_dict['Time [s]'])
        output_dict = scale_and_rename_channels(output_dict)
        if out_file_format == 1:
            export_to_OF_ASCII(output_dict, directory=None, filename=file_path, first_chunk=first_chunk)
        elif out_file_format == 2:
            export_to_OF_Binary(output_dict, outfilename=file_path.replace('.out', '.outb'))
        print(f"Simulation results of {sim_name} successfully written to {file_path}.", flush=True)
    else:
        output_dict['qblade_failed [-]'] = np.ones_like(output_dict['Time [s]'])
        output_dict = scale_and_rename_channels(output_dict)
        if out_file_format == 1:
            export_to_OF_ASCII(output_dict, directory=None, filename=file_path, first_chunk=first_chunk)
        elif out_file_format == 2:
            export_to_OF_Binary(output_dict, outfilename=file_path.replace('.out', '.outb'))
        print(f"Simulation results for {sim_name} successfully written to {file_path} (up to the point of failure).", flush=True)
    
    if 'True' in store_qprs:
        output_file = f"{sim_out_name}_completed.qpr".encode('ASCII')
        QBLIB.storeProject(output_file)
    
    QBLIB.closeInstance()
    del QBLIB.lib

def run_with_retry(QBlade_dll, QBLADE_runDirectory, sim, n_dt, channels, no_structure, store_qprs, store_from, chunk_size, out_file_format, max_retries=2, delay=2):
    attempt = 0
    while attempt < max_retries:
        try:
            print(f"Running simulation attempt {attempt + 1} for {sim}...")
            qblade_sil(QBlade_dll, QBLADE_runDirectory, sim, n_dt, channels, no_structure, store_qprs, store_from, chunk_size, out_file_format)
            return  # Exit if successful
        except Exception as e:
            print(f"Simulation attempt {attempt + 1} failed with exception: {e}")
            attempt += 1
            if attempt < max_retries:
                print("Retrying...")
                time.sleep(delay)
            else:
                print(f"Max retries reached for {sim}. Moving on.")

def run_qblade_sil(QBlade_dll, QBLADE_runDirectory, channels, n_dt, number_of_workers, no_structure, store_qprs, store_from, chunk_size, out_file_format):
    
    clear_and_delete_temp(QBlade_dll) # delete TEMP folder within QBlade directory to prevent unnecessary data clogging

    simulations = [os.path.join(QBLADE_runDirectory, f) for f in os.listdir(QBLADE_runDirectory) if f.endswith('.sim')]

    with concurrent.futures.ProcessPoolExecutor(max_workers=number_of_workers) as executor:
        futures = []
        for sim in simulations:
            time.sleep(1)  # Introduce a one-second pause before submitting the next task
            futures.append(
                executor.submit(run_with_retry, QBlade_dll, QBLADE_runDirectory, sim, n_dt, channels, no_structure, store_qprs, store_from, chunk_size, out_file_format)
            )
        for future in concurrent.futures.as_completed(futures):
            try:
                future.result()
            except Exception as e:
                print(f"Simulation failed after retrying with exception: {e}")

def export_to_OF_Binary(data, outfilename):
    """
    Writes a results file in OpenFAST Binary format.
    
    Parameters:
    - data: A DataFrame with channel names and data. Channel names are in the format 'ChannelName [Unit]'.
    - filename: The path to the output binary file.
    - description: A description string to be written in the header.
    """

    FileID = 2
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
    
    # Scaling Parameters
    Int32Max = np.float64(65535.0)                          
    Int32Min = np.float64(-65536.0)                         
    Int32Rang = np.float64(Int32Max-Int32Min)               
    IntMax = np.float64(32757.0)                            
    IntMin = np.float64(-32768.0)                           
    IntRang = np.float64(IntMax-IntMin)                     

    # Time parameters
    Time = np.float64(Chans.Time)
    TimeMax = Time.max()                                    
    TimeMin = Time.min()                                    
    TimeOff = 0                                             
    TimeScl = 0                                             
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
    TmpTimeArray = np.zeros(NT, dtype=np.int32)                                               
    ChanNameASCII = []                                      
    ChanUnitASCII = []                                      

    # FileID = 1 is with time, FileID =2 is without time
    if FileID == 1:
        TmpTimeArray = np.zeros(len(Time), dtype=np.int32)

    # for Name in ChansMod.columns:
    LenName = Chans.columns.str.len().max()
    for Name in Chans.columns:
        ChanNameASCII.append(Name.ljust(LenName).encode('utf-8'))

    LenUnit = len(max(ChanUnit, key=len))
    for Unit in ChanUnit:
        ChanUnitASCII.append(Unit.ljust(LenUnit).encode('latin_1'))

    if PackingData is None:
        for i in range(len(ColMax)):
            if ColMax[i] == ColMin[i]:
                ColScl.append(np.float32(1))
            else:
                ColScl.append(np.float32(IntRang/(ColMax[i] - ColMin[i])))

            ColOff.append(np.float32(IntMin-ColScl[i]*ColMin[i]))

        if FileID == 1:
            if TimeMax == TimeMin:
                TimeScl = np.float64(1)
            else:
                TimeScl = Int32Rang/np.float64((TimeMax-TimeMin))
            TimeOff = Int32Min - TimeScl*np.float64((TimeMin))
            
    else:
        ColScl = PackingData['ColScl']
        ColOff = PackingData['ColOff']
        TimeScl = PackingData['TimeScl']
        TimeOff = PackingData['TimeOff']

    TempFrame = ChansMod.copy()


    for j in range(NumOutChans):
        TempFrame[TempFrame.columns[j]] = ColScl[j] * TempFrame[TempFrame.columns[j]] + ColOff[j]

    TmpOutArray = np.clip((TempFrame.values.flatten()), IntMin, IntMax).astype(np.int16)

    # Pack the time into 32-bit integers
    if FileID == 1:
        TmpTimeArray = np.clip(TimeScl*Time+TimeOff, Int32Min, Int32Max).astype(np.int32)

    outfile = open(outfilename, 'wb')

    outfile.write(st.pack('h',np.int16(FileID)))
    outfile.write(st.pack('i',np.int32(NumOutChans)))
    outfile.write(st.pack('i',np.int32(NT)))

    if FileID == 1:
        outfile.write(st.pack('d',np.float64(TimeScl)))
        outfile.write(st.pack('d',np.float64(TimeOff)))
    else:
        outfile.write(st.pack('d',np.float64(TimeOut1)))
        outfile.write(st.pack('d',np.float64(TimeIncrement)))

    outfile.write(np.array(ColScl, dtype=np.float32))
    outfile.write(np.array(ColOff, dtype=np.float32))

    outfile.write(np.int32(LenDesc))

    outfile.write(DescStr.encode('utf-8'))
    outfile.write(np.array(ChanNameASCII))
    outfile.write(np.array(ChanUnitASCII))

    if FileID == 1:
        outfile.write(TmpTimeArray)

    outfile.write(TmpOutArray)
    outfile.close()

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
        # channels.append(split_string[0].replace(' ','_')) # This would be required to make the result file compatible with PDAP
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

    if first_chunk:  # Write header only for the first chunk
        with open(filename, 'w') as f:  # Open in 'w' mode for the first chunk
            f.write('---------------------------------------- file generated with WEIS QBlade API ----------------------------------------\n')
            f.write('Results are written in OpenFAST ASCII (.out) format\n')
            f.write(os.path.basename(filename)) # write the filename
            f.write("\n \n \n \n")
            data_string = dataframe.to_csv(sep='\t', index=False, header=True, lineterminator='\n')
            f.write(data_string)

    else:  # Append data for subsequent chunks, skipping the header and units row
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
    channels_str = sys.argv[3]
    n_dt = int(sys.argv[4])
    number_of_workers = int(sys.argv[5])
    no_structure = sys.argv[6]
    store_qprs = sys.argv[7]
    store_from =  float(sys.argv[8])
    chunk_size =  float(sys.argv[9])
    out_file_format =  float(sys.argv[10])

    # required inputs are converted back into the datatype that we need
    channels = channels_str.split(',') #convert back to list

    run_qblade_sil(QBlade_dll, QBLADE_runDirectory, channels, n_dt, number_of_workers, no_structure, store_qprs, store_from, chunk_size, out_file_format)