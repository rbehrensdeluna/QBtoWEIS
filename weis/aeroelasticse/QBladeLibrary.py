
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

class QBladeLibrary:

    def __init__(self, shared_lib_path):
        
        try:
            self.lib = CDLL(shared_lib_path)
            print("Successfully loaded ", shared_lib_path)
        except Exception as e:
            print("Could not load the file ", shared_lib_path)
            print(e)
            return
            
        #setting the library Path, so that the Library knows about its location!
        self.lib.setLibraryPath(shared_lib_path.encode('utf-8')) #setting the library Path, so that the DLL knows about its location!
        
        #here the imported functions are defined
        
        self.loadProject = self.lib.loadProject
        self.loadProject.argtype = c_char_p
        self.loadProject.restype = c_void_p
        
        self.loadSimDefinition = self.lib.loadSimDefinition
        self.loadSimDefinition.argtype = c_char_p
        self.loadSimDefinition.restype = c_void_p

        self.setOmpNumThreads = self.lib.setOmpNumThreads
        self.setOmpNumThreads.argtype = [c_int]
        self.setOmpNumThreads.restype = c_void_p
        
        self.getCustomData_at_num = self.lib.getCustomData_at_num
        self.getCustomData_at_num.argtypes = [c_char_p, c_double, c_int]
        self.getCustomData_at_num.restype = c_double
        
        self.getCustomSimulationData = self.lib.getCustomSimulationData
        self.getCustomSimulationData.argtype = c_char_p
        self.getCustomSimulationData.restype = c_double
        
        self.getWindspeed = self.lib.getWindspeed
        self.getWindspeed.argtypes = [c_double, c_double, c_double, c_double * 3]
        self.getWindspeed.restype = c_void_p
        
        self.getWindspeedArray = self.lib.getWindspeedArray
        self.getWindspeedArray.argtypes = [POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int]
        self.getWindspeedArray.restype = c_void_p
        
        self.storeProject = self.lib.storeProject
        self.storeProject.argtype = c_char_p
        self.storeProject.restype = c_void_p
        
        self.setLibraryPath = self.lib.setLibraryPath
        self.setLibraryPath.argtype = c_char_p
        self.setLibraryPath.restype = c_void_p
        
        self.setLogFile = self.lib.setLogFile
        self.setLogFile.argtype = c_char_p
        self.setLogFile.restype = c_void_p
        
        self.createInstance = self.lib.createInstance
        self.createInstance.argtypes = [c_int, c_int]
        self.createInstance.restype = c_void_p
        
        self.closeInstance = self.lib.closeInstance
        self.closeInstance.restype = c_void_p
        
        self.addTurbulentWind = self.lib.addTurbulentWind
        self.addTurbulentWind.argtypes = [c_double, c_double, c_double, c_double, c_int, c_double, c_double, c_char_p, c_char_p, c_int, c_double, c_double, c_bool]
        self.addTurbulentWind.restype = c_void_p
        
        self.setExternalAction = self.lib.setExternalAction
        self.setExternalAction.argtypes = [c_char_p, c_char_p, c_double, c_double, c_char_p, c_bool, c_int]
        self.setExternalAction.restype = c_void_p
        
        self.loadTurbulentWindBinary = self.lib.loadTurbulentWindBinary
        self.loadTurbulentWindBinary.argtype = c_char_p
        self.loadTurbulentWindBinary.restype = c_void_p
        
        self.setTimestepSize = self.lib.setTimestepSize
        self.setTimestepSize.argtype = c_double
        self.setTimestepSize.restype = c_void_p
        
        self.setInitialConditions_at_num = self.lib.setInitialConditions_at_num
        self.setInitialConditions_at_num.argtypes = [c_double, c_double, c_double, c_double, c_int]
        self.setInitialConditions_at_num.restype = c_void_p
        
        self.setRPMPrescribeType_at_num = self.lib.setRPMPrescribeType_at_num
        self.setRPMPrescribeType_at_num.argtypes = [c_int, c_int]
        self.setRPMPrescribeType_at_num.restype = c_void_p
        
        self.setRampupTime = self.lib.setRampupTime
        self.setRampupTime.argtype = c_double
        self.setRampupTime.restype = c_void_p
        
        self.setTurbinePosition_at_num = self.lib.setTurbinePosition_at_num
        self.setTurbinePosition_at_num.argtypes = [c_double, c_double, c_double, c_double, c_double, c_double, c_int]
        self.setTurbinePosition_at_num.restype = c_void_p
        
        self.getTowerBottomLoads_at_num = self.lib.getTowerBottomLoads_at_num
        self.getTowerBottomLoads_at_num.argtypes = [c_double * 6, c_int]
        self.getTowerBottomLoads_at_num.restype = c_void_p
        
        self.initializeSimulation = self.lib.initializeSimulation
        self.initializeSimulation.restype = c_void_p
        
        self.advanceTurbineSimulation = self.lib.advanceTurbineSimulation
        self.advanceTurbineSimulation.restype = c_void_p
        
        self.advanceController_at_num = self.lib.advanceController_at_num
        self.advanceController_at_num.argtypes = [c_double * 5, c_int]
        self.advanceController_at_num.restype = c_void_p
        
        self.setDebugInfo = self.lib.setDebugInfo
        self.setDebugInfo.argtype = c_bool
        self.setDebugInfo.restype = c_void_p
        
        self.setUseOpenCl = self.lib.setUseOpenCl
        self.setUseOpenCl.argtype = c_bool
        self.setUseOpenCl.restype = c_void_p
        
        self.setGranularDebug = self.lib.setGranularDebug
        self.setGranularDebug.argtypes = [c_bool, c_bool, c_bool, c_bool, c_bool]
        self.setGranularDebug.restype = c_void_p
        
        self.setControlVars_at_num = self.lib.setControlVars_at_num
        self.setControlVars_at_num.argtypes = [c_double * 5, c_int]
        self.setControlVars_at_num.restype = c_void_p
        
        self.getTurbineOperation_at_num = self.lib.getTurbineOperation_at_num
        self.getTurbineOperation_at_num.argtypes = [c_double * 41, c_int]
        self.getTurbineOperation_at_num.restype = c_void_p
        
        self.setPowerLawWind = self.lib.setPowerLawWind
        self.setPowerLawWind.argtypes = [c_double, c_double, c_double, c_double, c_double]
        self.setPowerLawWind.restype = c_void_p
        
        self.runFullSimulation = self.lib.runFullSimulation
        self.runFullSimulation.restype = c_void_p
