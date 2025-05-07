from ctypes import *
from typing import Dict, Any

class QBladeLibrary:
    def __init__(self, shared_lib_path: str):
        """Initialize and load the QBlade shared library."""
        self.lib_path = shared_lib_path
        self.lib = None

        # Define all functions with argument types and return types
        self.functions: Dict[str, Dict[str, Any]] = {
            "createInstance": {"argtypes": [c_int, c_int], "restype": c_bool},
            "closeInstance": {"argtypes": None, "restype": c_void_p},
            "loadProject": {"argtypes": [c_char_p], "restype": c_void_p},
            "loadSimDefinition": {"argtypes": [c_char_p], "restype": c_void_p},
            "setOmpNumThreads": {"argtypes": [c_int], "restype": c_void_p},
            "getCustomData_at_num": {"argtypes": [c_char_p, c_double, c_int], "restype": c_double},
            "getCustomSimulationTimeData": {"argtypes": [c_char_p], "restype": c_double},
            "getWindspeed": {"argtypes": [c_double, c_double, c_double, POINTER(c_double * 3)], "restype": c_void_p},
            "getWindspeedArray": {"argtypes": [POINTER(c_double), POINTER(c_double), POINTER(c_double),POINTER(c_double), POINTER(c_double), POINTER(c_double), c_int],"restype": c_void_p,},
            "storeProject": {"argtypes": [c_char_p], "restype": c_void_p},
            "exportResults": {"argtypes": [c_int, c_char_p, c_char_p, c_char_p], "restype": c_void_p},
            "setLibraryPath": {"argtypes": [c_char_p], "restype": c_void_p},
            "setLogFile": {"argtypes": [c_char_p], "restype": c_void_p},
            "addTurbulentWind": {"argtypes": [c_double, c_double, c_double, c_double, c_int, c_double,c_double, c_char_p, c_char_p, c_int, c_double, c_double, c_bool,],"restype": c_void_p,},
            "setExternalAction": {"argtypes": [c_char_p, c_char_p, c_double, c_double, c_char_p, c_bool, c_int],"restype": c_void_p,},
            "setMooringStiffness": {"argtypes": [c_double, c_double, c_int, c_int], "restype": c_void_p},
            "loadTurbulentWindBinary": {"argtypes": [c_char_p], "restype": c_void_p},
            "setTimestepSize": {"argtypes": [c_double], "restype": c_void_p},
            "setInitialConditions_at_num": {"argtypes": [c_double, c_double, c_double, c_double, c_int],"restype": c_void_p,},
            "setRPMPrescribeType_at_num": {"argtypes": [c_int, c_int], "restype": c_void_p},
            "setRPM_at_num": {"argtypes": [c_double, c_int], "restype": c_void_p},
            "setRampupTime": {"argtypes": [c_double], "restype": c_void_p},
            "setTurbinePosition_at_num": {"argtypes": [c_double, c_double, c_double, c_double, c_double, c_double, c_int],"restype": c_void_p,},
            "getTowerBottomLoads_at_num": {"argtypes": [POINTER(c_double * 6), c_int], "restype": c_void_p},
            "initializeSimulation": {"argtypes": None, "restype": c_void_p},
            "advanceTurbineSimulation": {"argtypes": None, "restype": c_bool},
            "advanceController_at_num": {"argtypes": [POINTER(c_double * 5), c_int], "restype": c_void_p},
            "setDebugInfo": {"argtypes": [c_bool], "restype": c_void_p},
            "setUseOpenCl": {"argtypes": [c_bool], "restype": c_void_p},
            "setGranularDebug": {"argtypes": [c_bool, c_bool, c_bool, c_bool, c_bool], "restype": c_void_p},
            "setControlVars_at_num": {"argtypes": [POINTER(c_double * 5), c_int], "restype": c_void_p},
            "getTurbineOperation_at_num": {"argtypes": [POINTER(c_double * 41), c_int], "restype": c_void_p},
            "setPowerLawWind": {"argtypes": [c_double, c_double, c_double, c_double, c_double], "restype": c_void_p},
            "runFullSimulation": {"argtypes": None, "restype": c_bool},
            "setAutoCleanup": {"argtypes": [c_bool], "restype": c_void_p},
        }
        
        # Automatically load the library
        self.load_library()

    def load_library(self):
        """Load the shared library and dynamically bind all functions."""
        try:
            self.lib = CDLL(self.lib_path)
            print(f"Successfully loaded library from: {self.lib_path}")
        except Exception as e:
            raise RuntimeError(f"Could not load the library at {self.lib_path}: {e}")

        # Bind functions dynamically
        for func_name, config in self.functions.items():
            try:
                func = getattr(self.lib, func_name)
                func.argtypes = config.get("argtypes")
                func.restype = config.get("restype")
                setattr(self, func_name, func)  # Bind the function to the instance
            except AttributeError as e:
                raise RuntimeError(f"Failed to bind function '{func_name}': {e}")

        # Call setLibraryPath after the library is loaded
        try:
            self.setLibraryPath(self.lib_path.encode('utf-8'))
            print(f"Library path set to: {self.lib_path}")
        except Exception as e:
            raise RuntimeError(f"Failed to set library path: {e}")

    def unload(self):
        
        # Close the QBlade instance if it exists
        try:
            self.closeInstance()
            print("QBlade instance closed.")
        except Exception as e:
            print(f"Warning: Failed to close QBlade instance: {e}")
        
        # Clean up resources and unload the library
        if self.lib:
            del self.lib
            self.lib = None
            print("Library unloaded successfully.")
