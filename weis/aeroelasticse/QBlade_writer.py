
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
import copy
import random
import time
import operator
import numpy as np
from functools import reduce
import logging
from ruamel.yaml import YAML

try:
    from rosco.toolbox import utilities as ROSCO_utilities
    ROSCO = True
except:
    ROSCO = False

logger = logging.getLogger("wisdem/weis") 

class InputWriter_QBlade(object):

    def __init__(self):
        
        self.QBLADE_namingOut = None    #Master QBlade file
        self.QBLADE_runDirectory = None #Output directory
        self.qb_vt = {}
        self.qb_update = {}
        self.turbsim_params = None
        self.store_turbines = False

    def execute(self):
        if not os.path.exists(self.QBLADE_runDirectory):
            os.makedirs(self.QBLADE_runDirectory)
        
        if self.qb_vt['QSim']['Simulate']:
            if not os.path.isdir(os.path.join(self.QBLADE_runDirectory,self.QBLADE_namingOut)):
                try:
                    os.makedirs(os.path.join(self.QBLADE_runDirectory,self.QBLADE_namingOut))
                except:
                    try:
                        time.sleep(random.random())
                        if not os.path.isdir(os.path.join(self.QBLADE_runDirectory,self.QBLADE_namingOut)):
                            os.makedirs(os.path.join(self.QBLADE_runDirectory,self.QBLADE_namingOut))
                    except:
                        print("Error tring to make '%s'!"%os.path.join(self.QBLADE_runDirectory,self.QBLADE_namingOut))
            self.turbine_directory = os.path.join(self.QBLADE_runDirectory, self.QBLADE_namingOut)
        else:
            self.turbine_directory = self.QBLADE_runDirectory

        # aerodynamic defitnion
        self.write_aero_dir()

        # structural definition
        self.write_structure_dir()

        # controller
        if self.qb_vt['Turbine']['CONTROLLERTYPE'] == 1:
            self.write_controller_dir()
            if 'DISCON_in' in self.qb_vt and ROSCO:
                self.write_DISCON_in()

        # write turbine definition file
        self.write_turbine_def()

        if self.qb_vt['QSim']['wave_flag'] and self.qb_vt['QSim']['ISOFFSHORE'] == 1 :
            self.write_wave_file()

        if self.qb_vt['QSim']['WNDTYPE'] and not self.qb_vt['QSim']['DLCGenerator']:
            self.write_turbsim_input()

        # write simulation setup file
        if self.qb_vt['QSim']['Simulate']:
            self.write_simulation_setup()

    def write_aero_dir(self):
        # create "Aero" subfolder and place .bld, .plr and Airfoil subdirecrie in it
        if not os.path.isdir(os.path.join(self.turbine_directory,'Aero')):
            try:
                os.makedirs(os.path.join(self.turbine_directory,'Aero'))
            except:
                try:
                    time.sleep(random.random())
                    if not os.path.isdir(os.path.join(self.turbine_directory,'Aero')):
                        os.makedirs(os.path.join(self.turbine_directory,'Aero'))
                except:
                    print("Error tring to make '%s'!"%os.path.join(self.turbine_directory,'Aero'))
            
        self.write_airfoils_dir() # create airfoil subdirectory
        self.write_polar_files() # write polar files

        self.qb_vt['Aero']['BldFile'] = os.path.join('Aero', self.QBLADE_namingOut + '.bld')
        bld_file = os.path.join(self.turbine_directory, self.qb_vt['Aero']['BldFile'])
        object_lenght = 30 
        keyword_length = 25
        with open(bld_file, 'w') as f:
            f.write('---------------------------------------- file generated with WEIS QBlade API ----------------------------------------\n')
            f.write('\n')
            f.write('---------------------------------------- QBlade Blade Definition File ----------------------------------------\n')
            f.write('\n')

            f.write('---------------------------------------- Object Name ----------------------------------------\n')
            f.write(f"{str(self.qb_vt['Main']['TurbineName']+'_BLADE '):<{object_lenght}}{'OBJECTNAME':<{keyword_length}} - the name of the blade object\n")
            f.write('\n')

            f.write('---------------------------------------- Parameters ----------------------------------------\n')
            f.write(f"{str(self.qb_vt['Aero']['ROTORTYPE']):<{object_lenght}}{'ROTORTYPE':<{keyword_length}} - the rotor type\n")
            f.write(f"{str(int(self.qb_vt['Aero']['INVERTEDFOILS'])):<{object_lenght}}{'INVERTEDFOILS':<{keyword_length}} - invert the airfoils? 0 - NO, 1 - YES (only VAWT)\n")
            f.write(f"{str(self.qb_vt['Main']['NUMBLD']):<{object_lenght}}{'NUMBLADES':<{keyword_length}} - number of blades\n")
            f.write('\n')

            f.write('---------------------------------------- Blade Data ----------------------------------------\n')
            f.write('POS [m]         CHORD [m]       TWIST [deg]     OFFSET_X [m]    OFFSET_Y [m]    P_AXIS [-]      POLAR_FILE\n')
            BlPos      = self.qb_vt['Aero']['BlPos']
            BlTwist    = self.qb_vt['Aero']['BlTwist']
            BlChord    = self.qb_vt['Aero']['BlChord']
            BlXOffset  = self.qb_vt['Aero']['XOffset']
            BlYOffset  = self.qb_vt['Aero']['YOffset']
            BlPaxis    = self.qb_vt['Aero']['Paxis']
            AflFile = [os.path.basename(plr) for plr in self.qb_vt['Aero']['PlrNames']]
            for Pos, Chord, Twist, XOffset, YOffset, Paxis, file in zip(BlPos, BlChord, BlTwist, BlXOffset, BlYOffset, BlPaxis, AflFile):
                f.write('{:0.8e} {: 5.8e} {: 5.8e} {: 5.8e} {: 5.8e} {: 5.8e}  {:s}\n'.format(Pos, Chord, Twist, XOffset, YOffset, Paxis, file))

    def write_structure_dir(self):
        # create "Structure" subfolder and place main, tower and blade file in it
        if not os.path.isdir(os.path.join(self.turbine_directory,'Structure')):
            try:
                os.makedirs(os.path.join(self.turbine_directory,'Structure'))
            except:
                try:
                    time.sleep(random.random())
                    if not os.path.isdir(os.path.join(self.turbine_directory,'Structure')):
                        os.makedirs(os.path.join(self.turbine_directory,'Structure'))
                except:
                    print("Error tring to make '%s'!"%os.path.join(self.turbine_directory,'Structure'))
        
        # write structure input files
        self.write_tower_file()
        if not self.qb_vt['Blade_6x6']:
            self.write_blade_files()
        else:
            self.write_blade_6x6_files()
        if self.qb_vt['QSim']['ISOFFSHORE'] == 1:
            self.write_sub_def()
        self.write_main_file()
        
    def write_airfoils_dir(self):
        # create "Airfoils" subfolder in "Aero" subfolder
        airfoils_dir = os.path.join(self.turbine_directory, 'Aero', 'Airfoils')
        if not os.path.isdir(airfoils_dir):
            os.makedirs(airfoils_dir)
        
        self.qb_vt['Aero']['AirfoilNames'] = ['']*len(self.qb_vt['Aero']['af_data'])
        for airfoil in range(len(self.qb_vt['Aero']['af_coord'])):
            self.qb_vt['Aero']['AirfoilNames'][airfoil] = os.path.join('Aero', 'Airfoils', self.QBLADE_namingOut + '_airfoil_coordinates_%02d.afl'%airfoil)
            airfoil_file = os.path.join(self.turbine_directory, self.qb_vt['Aero']['AirfoilNames'][airfoil])

            with open(airfoil_file, 'w') as f:
                f.write('---------------------------------------- Airfoil Coordinates generated with WEIS QBlade API ----------------------------------\n')
                f.write(self.QBLADE_namingOut + '_airfoil_coordinates_%02d.dat'%airfoil + '\n')
                Xcoord = self.qb_vt['Aero']['af_coord'][airfoil]['x']
                Ycoord = self.qb_vt['Aero']['af_coord'][airfoil]['y']
                for x, y in zip(Xcoord, Ycoord):
                    f.write('{:.4f} {:.4f}\n'.format(x, y))

    def write_polar_files(self):
        self.qb_vt['Aero']['NumPlrFiles'] = len(self.qb_vt['Aero']['af_data'])
        self.qb_vt['Aero']['PlrNames'] = ['']*self.qb_vt['Aero']['NumPlrFiles']

        object_length = 60
        keyword_length = 15

        for polar in range(self.qb_vt['Aero']['NumPlrFiles']):
            self.qb_vt['Aero']['PlrNames'][polar] = os.path.join('Aero', self.QBLADE_namingOut + '_polar_%02d.plr'%polar)
            plr_file = os.path.join(self.turbine_directory, self.qb_vt['Aero']['PlrNames'][polar])
            
            with open(plr_file, 'w') as f:
                
                tab = 0 # for now only one airfoil table 

                f.write('---------------------------------------- Polar file generated with WEIS QBlade API ----------------------------------\n\n')
                f.write('---------------------------------------- Object Names ----------------------------------\n')
                f.write(f"{polar:<{object_length}}{' POLARNAME':<{keyword_length}} - the polar name\n")
                f.write(f"{os.path.relpath(self.qb_vt['Aero']['AirfoilNames'][polar], 'Aero'):<{object_length}}{' FOILNAME':<{keyword_length}} - the airfoil name to which the polar(s) belong\n")
                f.write('\n')

                f.write('---------------------------------------- Parameters ----------------------------------\n')
                f.write(f"{self.qb_vt['Aero']['rthick'][polar]*100.:<{object_length}}{' THICKNESS':<{keyword_length}} - the thickness of the corresponding airfoil\n")
                f.write(f"{0:<{object_length}}{' ISDECOMPOSED':<{keyword_length}} - is the polar decomposed (add Cl_Sep, Cl_att and f_st columns)\n") # for now not possible
                f.write(f"{'REYNOLDS            '}{'{: .6e}'.format(self.qb_vt['Aero']['af_data'][polar][tab]['Re']):<{20}} - the Reynolds number for the imported polar\n") # for now only one is possible not possible
                # f.write(f"{str(self.qb_vt['Aero']['airfoils_Re']):<{object_length}}{' REYNOLDS':<{keyword_length}} - the list of Reynolds numbers for the imported polars)\n")
                f.write('\n')

                f.write('---------------------------------------- Polar Data ----------------------------------\n')
                f.write('AOA [deg]        CL [-]          CD [-]          CM [-]\n')
                polar_map = [self.qb_vt['Aero']['InCol_Alfa'], self.qb_vt['Aero']['InCol_Cl'], self.qb_vt['Aero']['InCol_Cd'], self.qb_vt['Aero']['InCol_Cm']]
                # polar_map.remove(0)
                polar_map = [i-1 for i in polar_map]

                
                alpha = np.asarray(self.qb_vt['Aero']['af_data'][polar][tab]['Alpha'])
                cl = np.asarray(self.qb_vt['Aero']['af_data'][polar][tab]['Cl'])
                cd = np.asarray(self.qb_vt['Aero']['af_data'][polar][tab]['Cd'])
                cm = np.asarray(self.qb_vt['Aero']['af_data'][polar][tab]['Cm'])

                if alpha[0] != -180.:
                    print('Airfoil number ' + str(polar) + ' tab number ' + str(tab) + ' has the min angle of attack different than -180 deg, and equal to ' + str(alpha[0]) + ' deg. This is changed to -180 deg now.')
                    alpha[0] = -180.
                if alpha[-1] != 180.:
                    print('Airfoil number ' + str(polar) + ' tab number ' + str(tab) + ' has the max angle of attack different than 180 deg, and equal to ' + str(alpha[0]) + ' deg. This is changed to 180 deg now.')
                    alpha[-1] = 180.
                if cl[0] != cl[-1]:
                    print('Airfoil number ' + str(polar) + ' tab number ' + str(tab) + ' has the lift coefficient different between +-180 deg. This is changed to be the same now.')
                    cl[0] = cl[-1]
                if cd[0] != cd[-1]:
                    print('Airfoil number ' + str(polar) + ' tab number ' + str(tab) + ' has the drag coefficient different between +-180 deg. This is changed to be the same now.')
                    cd[0] = cd[-1]
                if cm[0] != cm[-1]:
                    print('Airfoil number ' + str(polar) + ' tab number ' + str(tab) + ' has the moment coefficient different between +-180 deg. This is changed to be the same now.')
                    cm[0] = cm[-1]

                if self.qb_vt['Aero']['InCol_Cm'] == 0:
                    cm = np.zeros_like(cl)

                polar = np.column_stack((alpha, cl, cd, cm))
                polar = polar[:,polar_map]

                for row in polar:
                    f.write(' '.join(['{: .8e}'.format(val) for val in row])+'\n') 

    def write_main_file(self):
        # Write main file
        # self.write_qblade_input()
        # self.write_qblade_runfile()
        # self.write_qblade_batfile()
        self.qb_vt['Main']['MainFile'] = os.path.join('Structure', self.QBLADE_namingOut + '_Main.str')
        main_file = os.path.join(self.turbine_directory, self.qb_vt['Main']['MainFile'])
        blade_file = os.path.basename(self.qb_vt['Blade']['BladeFile']) # filename without the path
        tower_file = os.path.basename(self.qb_vt['Tower']['TowerFile']) # filename without the path
        if self.qb_vt['QSim']['ISOFFSHORE'] == 1:
            sub_file = os.path.basename(self.qb_vt['QBladeOcean']['SubFile']) # filename without the path
        else: 
            sub_file = ''

        object_lenght = 30 
        keyword_length = 25

        with open(main_file, 'w') as f:
            f.write('---------------------- file generated with WEIS QBlade API ----------------------\n')
            f.write('\n')
            f.write('---------------------- QBLADE STRUCTURAL MODEL INPUT FILE ----------------------\n')
            
            f.write(self.qb_vt['Main']['TurbineName'] + '\n')
            f.write('------------------------------- CHRONO PARAMETERS -------------------------\n')
            f.write(f"{str(self.qb_vt['Main']['GLBGEOEPS']):<{object_lenght}}{'GLBGEOEPS':<{keyword_length}} - Global geometry epsilon for node placement\n")
            f.write('\n')

            f.write('------------------------------- HAWT TURBINE CONFIGURATION ----------------\n')
            f.write(f"{str(self.qb_vt['Main']['PreCone']):<{object_lenght}}{'PRECONE':<{keyword_length}} - Rotor PreCone (deg) (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Main']['ShftTilt']):<{object_lenght}}{'SHFTTILT':<{keyword_length}} - Turbine Shaft Tilt (deg) (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Main']['OverHang']):<{object_lenght}}{'OVERHANG':<{keyword_length}} - Rotor Overhang (m) (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Main']['Twr2Shft']):<{object_lenght}}{'TWR2SHFT':<{keyword_length}} - Tower to Shaft distance (m) (HAWT only)\n")
            f.write('\n')

            f.write('------------------------------- MASS AND INERTIA --------------------------\n')
            f.write(f"{str(self.qb_vt['Main']['YawBrMass']):<{object_lenght}}{'YAWBRMASS':<{keyword_length}} - Yaw Bearing Mass (kg) (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Main']['NacMass']):<{object_lenght}}{'NACMASS':<{keyword_length}} - Nacelle Mass (kg) (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Main']['NacCMx'] ):<{object_lenght}}{'NACCMX':<{keyword_length}} - Downwind distance from the tower-top to the nacelle CM (m) (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Main']['NacCMy'] ):<{object_lenght}}{'NACCMY':<{keyword_length}} - Lateral  distance from the tower-top to the nacelle CM (m) (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Main']['NacCMz'] ):<{object_lenght}}{'NACCMZ':<{keyword_length}} - Vertical distance from the tower-top to the nacelle CM (m) (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Main']['NacYIner']):<{object_lenght}}{'NACYINER':<{keyword_length}} - Nacelle Yaw Inertia (kg*m^2) (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Main']['HubMass']):<{object_lenght}}{'HUBMASS':<{keyword_length}} - Hub Mass (kg)\n") 
            f.write(f"{str(self.qb_vt['Main']['HubIner']):<{object_lenght}}{'HUBINER':<{keyword_length}} - Hub Inertia (kg*m^2)\n")
            f.write('\n')  
            # f.write(str() + '\t\t\t XXXX     \t - \t YYYYYY\n')   

            f.write('------------------------------- DRIVETRAIN MODEL --------------------------\n')
            f.write(f"{str(self.qb_vt['Main']['GBRATIO']):<{object_lenght}}{'GBRATIO':<{keyword_length}} - gearbox ratio (N)\n")
            f.write(f"{str(self.qb_vt['Main']['GBOXEFF']):<{object_lenght}}{'GBOXEFF':<{keyword_length}} - gearbox efficiency (0-1)\n")
            f.write(f"{str(self.qb_vt['Main']['GENEFF']):<{object_lenght}}{'GENEFF':<{keyword_length}} - generator efficiency  (0-1)\n")
            f.write(f"{str(self.qb_vt['Main']['DRTRDOF']):<{object_lenght}}{'DRTRDOF':<{keyword_length}} - Model drivetrain dynamics (true / false)\n") 
            f.write(f"{str(self.qb_vt['Main']['GENINER']):<{object_lenght}}{'GENINER':<{keyword_length}} - Generator side (HSS) Inertia (kg*m^2)\n") 
            f.write(f"{str(self.qb_vt['Main']['DTTORSPR']):<{object_lenght}}{'DTTORSPR':<{keyword_length}} - Drivetrain torsional stiffness (N*m/rad)\n") 
            f.write(f"{str(self.qb_vt['Main']['DTTORDMP']):<{object_lenght}}{'DTTORDMP':<{keyword_length}} - Drivetrain torsional damping (N*m*s/rad)\n") 
            f.write('\n')

            f.write('------------------------------- BRAKE MODEL -------------------------------\n')
            f.write(f"{str(self.qb_vt['Main']['BRKTORQUE']):<{object_lenght}}{'BRKTORQUE':<{keyword_length}} - Maximum brake torque\n")
            f.write(f"{str(self.qb_vt['Main']['BRKDEPLOY']):<{object_lenght}}{'BRKDEPLOY':<{keyword_length}} - Brake deploy time (s) (only used with DTU style controllers)\n")
            f.write(f"{str(self.qb_vt['Main']['BRKDELAY']):<{object_lenght}}{'BRKDELAY':<{keyword_length}} - Brake delay time (s) (only used with DTU style controllers)\n")
            f.write('\n')

            f.write('------------------------------- SENSOR ERRORS -----------------------------\n')
            f.write(f"{str(self.qb_vt['Main']['ERRORYAW']):<{object_lenght}}{'ERRORYAW':<{keyword_length}} - Yaw error (deg) (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Main']['ERRORPITCH_1']):<{object_lenght}}{'ERRORPITCH_1':<{keyword_length}} - Pitch error blade1 (deg)\n")
            f.write(f"{str(self.qb_vt['Main']['ERRORPITCH_2']):<{object_lenght}}{'ERRORPITCH_2':<{keyword_length}} - Pitch error blade2 (deg)\n")
            f.write(f"{str(self.qb_vt['Main']['ERRORPITCH_3']):<{object_lenght}}{'ERRORPITCH_3':<{keyword_length}} - Pitch error blade3 (deg)\n")
            f.write('\n')

            f.write('------------------------------- BLADES ------------------------------------\n')
            f.write(f"{str(self.qb_vt['Main']['NUMBLD']):<{object_lenght}}{'NUMBLD':<{keyword_length}} - Number of blades\n")
            for blade in range(1, self.qb_vt['Main']['NUMBLD'] + 1):
                f.write(f"{str(blade_file+' '):<{object_lenght}}{'BLDFILE_' + str(blade) + '':<{keyword_length}} - Name of file containing properties for blade '+ str(blade) + '\n")
            f.write('\n')

            f.write('------------------------------- TOWER -------------------------------------\n')
            f.write(f"{str(self.qb_vt['Main']['TWRHEIGHT']):<{object_lenght}}{'TWRHEIGHT':<{keyword_length}} - Height of the tower (m)\n")
            f.write(f"{str(tower_file+' '):<{object_lenght}}{'TWRFILE':<{keyword_length}} - Name of file containing properties for the tower\n")
            f.write('\n')

            f.write('------------------------------- Substructure -------------------------------------\n')
            f.write(f"{str(sub_file+' '):<{object_lenght}}{'SUBFILE':<{keyword_length}} - Name of file containing properties for the substructure\n")
            f.write('\n')

            f.write('------------------------------- DATA OUTPUT TYPES -------------------------\n')
            f.write(f"{str(self.qb_vt['Main']['AER_OUT']):<{object_lenght}}{'AER_OUT':<{keyword_length}} - Aerodynamic output\n")
            f.write(f"{str(self.qb_vt['Main']['FOR_OUT']):<{object_lenght}}{'FOR_OUT':<{keyword_length}} - Structural output\n")
            f.write(f"{str(self.qb_vt['Main']['ROT_OUT']):<{object_lenght}}{'ROT_OUT':<{keyword_length}} - Rotor output\n")
            f.write(f"{str(self.qb_vt['Main']['MOM_OUT']):<{object_lenght}}{'MOM_OUT':<{keyword_length}} - Moments output\n")
            f.write(f"{str(self.qb_vt['Main']['DEF_OUT']):<{object_lenght}}{'DEF_OUT':<{keyword_length}} - Deflections output\n")
            f.write(f"{str(self.qb_vt['Main']['POS_OUT']):<{object_lenght}}{'POS_OUT':<{keyword_length}} - Positions output\n")
            f.write(f"{str(self.qb_vt['Main']['VEL_OUT']):<{object_lenght}}{'VEL_OUT':<{keyword_length}} - Velocities output\n")
            f.write(f"{str(self.qb_vt['Main']['ACC_OUT']):<{object_lenght}}{'ACC_OUT':<{keyword_length}} - Accelerations output\n")
            f.write(f"{str(self.qb_vt['Main']['LVE_OUT']):<{object_lenght}}{'LVE_OUT':<{keyword_length}} - Linear velocities output\n")
            f.write(f"{str(self.qb_vt['Main']['LAC_OUT']):<{object_lenght}}{'LAC_OUT':<{keyword_length}} - Linear accelerations output\n")
            f.write('\n')

            f.write('------------------------------- DATA OUTPUT LOCATIONS -----------------------------\n')
            for blade in range(1, self.qb_vt['Main']['NUMBLD'] + 1):
                blade_positions = self.qb_vt['Main']['BLD_' + str(blade) ]
                for position in blade_positions:
                    f.write(f'BLD_{blade}_{position:.2f} - output position of blade {blade} at {position * 100:.0f}% normalized radius\n')
                f.write('\n')
            
            tower_positions = self.qb_vt['Main']['TWR']
            for position in tower_positions:
                f.write(f'TWR_{position:.2f} - output position of the tower at {position * 100:.0f}% normalized radius\n')
            f.write('\n')

    def write_tower_file(self):
        self.qb_vt['Tower']['TowerFile'] = os.path.join('Structure', self.QBLADE_namingOut + '_Tower.str')
        tower_file = os.path.join(self.turbine_directory, self.qb_vt['Tower']['TowerFile'])

        object_lenght = 30 
        keyword_length = 25
        with open(tower_file, 'w') as f:
            f.write('---------------------- file generated with WEIS QBlade API ----------------------\n')
            f.write('\n')
            f.write('---------------------- QBLADE TOWER INPUT FILE ----------------------\n')

            f.write(f"{str(self.qb_vt['Tower']['RAYLEIGHDMP']):<{object_lenght}}RAYLEIGHDMP \n")
            f.write(f"{str(self.qb_vt['Tower']['STIFFTUNER']):<{object_lenght}}STIFFTUNER \n")
            f.write(f"{str(self.qb_vt['Tower']['MASSTUNER']):<{object_lenght}}MASSTUNER \n")
            f.write('\n')

            f.write(f"{str(self.qb_vt['Tower']['INTPTYPE']):<{object_lenght}}INTPTYPE 0-LINEAR; 1-AKIMA; 2-HERMITE; 3-C2SPLINE \n")
            f.write(f"{str(self.qb_vt['Tower']['BEAMTYPE']):<{object_lenght}}BEAMTYPE 0-EULER; 1-TIMOSHENKO; 2-TIMOSHENKO_FPM \n")
            f.write(f"{str(self.qb_vt['Tower']['DISCTYPE']):<{object_lenght}}DISCTYPE 0-LINEAR; 1-COSINE; 2-STRUCT; 3-AERO \n")
            f.write(f"{str(self.qb_vt['Tower']['DISC']):<{object_lenght}}DISC \n")
            f.write('\n')

            # TODO AddMasses
            f.write('LENFRACT_[-]    MASSD_[kg/m]    EIx_[N.m^2]     EIy_[N.m^2]     EA_[N]          GJ_[N.m^2]      GA_[N]          STRPIT_[deg]    KSX_[-]'
                    '         KSY_[-]         RGX_[-]         RGY_[-]         XCM_[-]         YCM_[-]         XCE_[-]         YCE_[-]         XCS_[-]         YCS_[-]'
                    '         DIA_[m]         CD_[-]''\n')

            LENFRACT   = self.qb_vt['Tower']['LENFRACT']
            MASSD      = self.qb_vt['Tower']['MASSD']
            EIx        = self.qb_vt['Tower']['EIx']
            EIy        = self.qb_vt['Tower']['EIy']
            EA         = self.qb_vt['Tower']['EA']
            GJ         = self.qb_vt['Tower']['GJ']
            GA         = self.qb_vt['Tower']['GA']
            STRPIT     = self.qb_vt['Tower']['STRPIT']
            KSX        = self.qb_vt['Tower']['KSX']
            KSY        = self.qb_vt['Tower']['KSY']
            RGX        = self.qb_vt['Tower']['RGX']
            RGY        = self.qb_vt['Tower']['RGY']
            XCM        = self.qb_vt['Tower']['XCM']
            YCM        = self.qb_vt['Tower']['YCM']
            XCE        = self.qb_vt['Tower']['XCE']
            YCE        = self.qb_vt['Tower']['YCE']
            XCS        = self.qb_vt['Tower']['XCS']
            YCS        = self.qb_vt['Tower']['YCS']
            DIA        = self.qb_vt['Tower']['DIA']
            CD         = self.qb_vt['Tower']['CD']

            for LENFRACT, MASSD, EIx, EIy, EA, GJ, GA, STRPIT, KSX, KSY, RGX, RGY, XCM, YCM, XCE, YCE, XCS, YCS, DIA, CD in zip(LENFRACT, MASSD, EIx, EIy, EA, GJ, GA, STRPIT, KSX, KSY, RGX, RGY, XCM, YCM, XCE, YCE, XCS, YCS, DIA, CD):
                f.write('{:2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e}\n'.format(LENFRACT, MASSD, EIx, EIy, EA, GJ, GA, STRPIT, KSX, KSY, RGX, RGY, XCM, YCM, XCE, YCE, XCS, YCS, DIA, CD))

            f.write('\n')
            f.write('RGBCOLOR\n')
            f.write('R	G	B\n')
            f.write('230	230	230\n')

    def write_blade_6x6_files(self):
        self.qb_vt['Blade']['BladeFile'] = os.path.join('Structure', self.QBLADE_namingOut + '_Blade.str')
        blade_file = os.path.join(self.turbine_directory, self.qb_vt['Blade']['BladeFile'])

        object_lenght = 30 
        keyword_length = 25
        with open(blade_file, 'w') as f:
            f.write('---------------------- file generated with WEIS QBlade API ----------------------\n')
            f.write('\n')
            f.write('---------------------- QBLADE BLADE FPM INPUT FILE ----------------------\n')
            f.write('\n')
            if not self.qb_vt['Blade']['USERAYLEIGHDMP_ANISO']:
                f.write(f"{str(self.qb_vt['Blade']['RAYLEIGHDMP']):<{object_lenght}}RAYLEIGHDMP \n")
            else:
                f.write(f"{' '.join(map(str, self.qb_vt['Blade']['RAYLEIGHDMP_ANISO'])):<{object_lenght}} RAYLEIGHDMP_ANISO \n")
            f.write(f"{str(self.qb_vt['Blade']['STIFFTUNER']):<{object_lenght}}STIFFTUNER \n")
            f.write(f"{str(self.qb_vt['Blade']['MASSTUNER']):<{object_lenght}}MASSTUNER \n")
            f.write('\n')
            
            f.write(f"{str(self.qb_vt['Blade']['INTPTYPE']):<{object_lenght}}INTPTYPE 0-LINEAR; 1-AKIMA; 2-HERMITE; 3-C2SPLINE \n")
            f.write(f"{str(self.qb_vt['Blade']['BEAMTYPE']):<{object_lenght}}BEAMTYPE 0-EULER; 1-TIMOSHENKO; 2-TIMOSHENKO_FPM \n")
            f.write(f"{str(self.qb_vt['Blade']['DISCTYPE']):<{object_lenght}}DISCTYPE 0-LINEAR; 1-COSINE; 2-STRUCT; 3-AERO \n")
            f.write(f"{str(self.qb_vt['Blade']['DISC']):<{object_lenght}}DISC \n")
            f.write('\n')

            f.write('LENFRACT_[-]  XCB_[-]       YCB_[-]       PITCH_[deg]   K11_[N]       K12_[N]       K13_[N]       K14_[Nm]      K15_[Nm]      K16_[Nm]      K22_[N]       K23_[N]       K24_[Nm]      K25_[Nm]      K26_[Nm]      K33_[N]       K34_[Nm]      K35_[Nm]      K36_[Nm]      K44_[Nm^2]    K45_[Nm^2]    K46_[Nm^2]    K55_[Nm^2]    K56_[Nm^2]    K66_[Nm^2]    M11_[kg]      M12_[kg]      M13_[kg]      M14_[kgm]     M15_[kgm]     M16_[kgm]     M22_[kg]      M23_[kg]      M24_[kgm]     M25_[kgm]     M26_[kgm]     M33_[kg]      M34_[kgm]     M35_[kgm]     M36_[kgm]     M44_[kgm^2]   M45_[kgm^2]   M46_[kgm^2]   M55_[kgm^2]   M56_[kgm^2]   M66_[kgm^2]\n')

            # Access the values from the self.qb_vt['Blade_6x6'] dictionary
            LENFRACT = self.qb_vt['Blade_6x6']['LENFRACT']
            XCB      = self.qb_vt['Blade_6x6']['XCB']
            YCB      = self.qb_vt['Blade_6x6']['YCB']
            PITCH    = self.qb_vt['Blade_6x6']['PITCH']
            
            # Access all the stiffness and inertia components from self.qb_vt
            K11, K12, K13, K14, K15, K16 = (self.qb_vt['Blade_6x6'][key] for key in ['K11', 'K12', 'K13', 'K14', 'K15', 'K16'])
            K22, K23, K24, K25, K26 = (self.qb_vt['Blade_6x6'][key] for key in ['K22', 'K23', 'K24', 'K25', 'K26'])
            K33, K34, K35, K36 = (self.qb_vt['Blade_6x6'][key] for key in ['K33', 'K34', 'K35', 'K36'])
            K44, K45, K46 = (self.qb_vt['Blade_6x6'][key] for key in ['K44', 'K45', 'K46'])
            K55, K56, K66 = (self.qb_vt['Blade_6x6'][key] for key in ['K55', 'K56', 'K66'])
            
            M11, M12, M13, M14, M15, M16 = (self.qb_vt['Blade_6x6'][key] for key in ['M11', 'M12', 'M13', 'M14', 'M15', 'M16'])
            M22, M23, M24, M25, M26 = (self.qb_vt['Blade_6x6'][key] for key in ['M22', 'M23', 'M24', 'M25', 'M26'])
            M33, M34, M35, M36 = (self.qb_vt['Blade_6x6'][key] for key in ['M33', 'M34', 'M35', 'M36'])
            M44, M45, M46 = (self.qb_vt['Blade_6x6'][key] for key in ['M44', 'M45', 'M46'])
            M55, M56, M66 = (self.qb_vt['Blade_6x6'][key] for key in ['M55', 'M56', 'M66'])

            # Loop over all radial stations using zip to combine the lists
            for LENFRACT, XCB, YCB, PITCH, K11, K12, K13, K14, K15, K16, K22, K23, K24, K25, K26, K33, K34, K35, K36, K44, K45, K46, K55, K56, K66, M11, M12, M13, M14, M15, M16, M22, M23, M24, M25, M26, M33, M34, M35, M36, M44, M45, M46, M55, M56, M66 in zip(
                LENFRACT, XCB, YCB, PITCH, K11, K12, K13, K14, K15, K16, K22, K23, K24, K25, K26, K33, K34, K35, K36, K44, K45, K46, K55, K56, K66, M11, M12, M13, M14, M15, M16, M22, M23, M24, M25, M26, M33, M34, M35, M36, M44, M45, M46, M55, M56, M66):
                
                # Write the formatted line into the file
                f.write('{:2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} '
                        '{: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} '
                        '{: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} '
                        '{: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} '
                        '{: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e}\n'.format(
                            LENFRACT, XCB, YCB, PITCH,
                            K11, K12, K13, K14, K15, K16,
                            K22, K23, K24, K25, K26,
                            K33, K34, K35, K36,
                            K44, K45, K46,
                            K55, K56, K66,
                            M11, M12, M13, M14, M15, M16,
                            M22, M23, M24, M25, M26,
                            M33, M34, M35, M36,
                            M44, M45, M46,
                            M55, M56, M66))
                
    def write_blade_files(self):
        self.qb_vt['Blade']['BladeFile'] = os.path.join('Structure', self.QBLADE_namingOut + '_Blade.str')
        blade_file = os.path.join(self.turbine_directory, self.qb_vt['Blade']['BladeFile'])

        object_lenght = 30 
        keyword_length = 25
        with open(blade_file, 'w') as f:
            f.write('---------------------- file generated with WEIS QBlade API ----------------------\n')
            f.write('\n')
            f.write('---------------------- QBLADE BLADE INPUT FILE ----------------------\n')
            f.write('\n')
            if not self.qb_vt['Blade']['USERAYLEIGHDMP_ANISO']:
                f.write(f"{str(self.qb_vt['Blade']['RAYLEIGHDMP']):<{object_lenght}}RAYLEIGHDMP \n")
            else:
                f.write(f"{' '.join(map(str, self.qb_vt['Blade']['RAYLEIGHDMP_ANISO'])):<{object_lenght}} RAYLEIGHDMP_ANISO \n")
            f.write(f"{str(self.qb_vt['Blade']['STIFFTUNER']):<{object_lenght}}STIFFTUNER \n")
            f.write(f"{str(self.qb_vt['Blade']['MASSTUNER']):<{object_lenght}}MASSTUNER \n")
            f.write('\n')

            f.write(f"{str(self.qb_vt['Blade']['INTPTYPE']):<{object_lenght}}INTPTYPE 0-LINEAR; 1-AKIMA; 2-HERMITE; 3-C2SPLINE \n")
            f.write(f"{str(self.qb_vt['Blade']['BEAMTYPE']):<{object_lenght}}BEAMTYPE 0-EULER; 1-TIMOSHENKO; 2-TIMOSHENKO_FPM \n")
            f.write(f"{str(self.qb_vt['Blade']['DISCTYPE']):<{object_lenght}}DISCTYPE 0-LINEAR; 1-COSINE; 2-STRUCT; 3-AERO \n")
            f.write(f"{str(self.qb_vt['Blade']['DISC']):<{object_lenght}}DISC \n")
            f.write('\n')

            # TODO AddMasses
            f.write('LENFRACT_[-]    MASSD_[kg/m]    EIx_[N.m^2]     EIy_[N.m^2]     EA_[N]          GJ_[N.m^2]      GA_[N]          STRPIT_[deg]    KSX_[-]'
                    '         KSY_[-]         RGX_[-]         RGY_[-]         XCM_[-]         YCM_[-]         XCE_[-]         YCE_[-]         XCS_[-]         YCS_[-]\n')
            
            LENFRACT   = self.qb_vt['Blade']['LENFRACT']
            MASSD      = self.qb_vt['Blade']['MASSD']
            EIx        = self.qb_vt['Blade']['EIx']
            EIy        = self.qb_vt['Blade']['EIy']
            EA         = self.qb_vt['Blade']['EA']
            GJ         = self.qb_vt['Blade']['GJ']
            GA         = self.qb_vt['Blade']['GA']
            STRPIT     = self.qb_vt['Blade']['STRPIT']
            KSX        = self.qb_vt['Blade']['KSX']
            KSY        = self.qb_vt['Blade']['KSY']
            RGX        = self.qb_vt['Blade']['RGX']
            RGY        = self.qb_vt['Blade']['RGY']
            XCM        = self.qb_vt['Blade']['XCM']
            YCM        = self.qb_vt['Blade']['YCM']
            XCE        = self.qb_vt['Blade']['XCE']
            YCE        = self.qb_vt['Blade']['YCE']
            XCS        = self.qb_vt['Blade']['XCS']
            YCS        = self.qb_vt['Blade']['YCS']

            for LENFRACT, MASSD, EIx, EIy, EA, GJ, GA, STRPIT, KSX, KSY, RGX, RGY, XCM, YCM, XCE, YCE, XCS, YCS in zip(LENFRACT, MASSD, EIx, EIy, EA, GJ, GA, STRPIT, KSX, KSY, RGX, RGY, XCM, YCM, XCE, YCE, XCS, YCS):
                f.write('{:2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e} {: 2.8e}\n'.format(LENFRACT, MASSD, EIx, EIy, EA, GJ, GA, STRPIT, KSX, KSY, RGX, RGY, XCM, YCM, XCE, YCE, XCS, YCS))

    def write_turbine_def(self): 
        self.qb_vt['Turbine']['TrbFile'] = os.path.join(self.QBLADE_namingOut + '.trb')
        trb_file = os.path.join(self.turbine_directory, self.qb_vt['Turbine']['TrbFile'])
        object_length = 30 
        keyword_length = 25

        if self.qb_vt['Turbine']['NOSTRUCTURE']:
            self.qb_vt['Main']['MainFile'] = ''

        if self.qb_vt['Turbine']['CONTROLLERTYPE'] == 0:
                self.qb_vt['Turbine']['CONTROLLERFILE'] = None
                self.qb_vt['Turbine']['DLL_InFile'] = None
        
        with open(trb_file, 'w') as f:
            f.write('---------------------------------------- file generated with WEIS QBlade API ----------------------------------------\n')
            f.write('\n')
            f.write('---------------------------------------- QBlade Turbine Definition File ----------------------------------------\n')
            f.write('\n\n\n')

            f.write('---------------------------------------- Object Name ----------------------------------------\n')
            f.write(f"{str(self.QBLADE_namingOut+'_TURBINE '):<{object_length}}{'OBJECTNAME':<{keyword_length}} - the name of the turbine object\n")
            f.write('\n')

            f.write('---------------------------------------- Rotor Definition ----------------------------------------\n')
            f.write(f"{str(self.qb_vt['Aero']['BldFile']+' '):<{object_length}}{'BLADEFILE':<{keyword_length}} - the path of the blade file that is used in this turbine definition\n")
            f.write(f"{str(self.qb_vt['Turbine']['TURBTYPE']):<{object_length}}{'TURBTYPE':<{keyword_length}} - the turbine type (0 = HAWT or 1 = VAWT)\n")
            f.write(f"{str(self.qb_vt['Main']['NUMBLD']):<{object_length}}{'NUMBLADES':<{keyword_length}} - the number of blades (a Structural Model overrides this value)\n")
            f.write(f"{str(self.qb_vt['Turbine']['ROTORCONFIG']):<{object_length}}{'ROTORCONFIG':<{keyword_length}} - the rotor configuration (0 = Upwind, 1 = Downwind)\n")
            f.write(f"{str(self.qb_vt['Turbine']['ROTATIONALDIR']):<{object_length}}{'ROTATIONALDIR':<{keyword_length}} - the direction of rotor rotation (0 = STANDARD or 1 = REVERSED)\n")
            f.write(f"{str(self.qb_vt['Turbine']['DISCTYPE']):<{object_length}}{'DISCTYPE':<{keyword_length}} - type of rotor discretization (0 = from Bladetable, 1 = linear, 2 = cosine)\n")
            f.write(f"{str(self.qb_vt['Turbine']['INTPTYPE']):<{object_length}}{'INTPTYPE':<{keyword_length}} - type of rotor interpolation (0 = linear, 1 = Akima splines) \n")
            f.write(f"{str(self.qb_vt['Turbine']['NUMPANELS']):<{object_length}}{'NUMPANELS':<{keyword_length}} - the number of aerodynamic panels per blade (unused if DISCTYPE = 0)\n")
            f.write('\n')

            f.write('----------------------------------------Turbine Geometry Parameters-------------------------------------------------\n')
            f.write('These values are only used if no Structural Model is defined for this Turbine, in case of a Structural Model the geometry is defined in the Structural Input Files!!\n')
            f.write(f"{str(self.qb_vt['Turbine']['OVERHANG']):<{object_length}}{'OVERHANG':<{keyword_length}} - the overhang of the rotor [m] (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Turbine']['SHAFTTILT']):<{object_length}}{'SHAFTTILT':<{keyword_length}} - the shaft tilt angle [deg] (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Turbine']['ROTORCONE']):<{object_length}}{'ROTORCONE':<{keyword_length}} - the rotor cone angle [deg] (HAWT only)\n")
            f.write(f"{str(self.qb_vt['Turbine']['CLEARANCE']):<{object_length}}{'CLEARANCE':<{keyword_length}} - the rotor clearance to ground [m] (VAWT only)\n")
            f.write(f"{str(self.qb_vt['Turbine']['XTILT']):<{object_length}}{'XTILT':<{keyword_length}} - the rotor x-tilt angle [deg] (VAWT only)\n")
            f.write(f"{str(self.qb_vt['Turbine']['YTILT']):<{object_length}}{'YTILT':<{keyword_length}} - the rotor y-tilt angle [deg] (VAWT only)\n")
            f.write(f"{str(self.qb_vt['Turbine']['TOWERHEIGHT']):<{object_length}}{'TOWERHEIGHT':<{keyword_length}} - the tower height [m]\n")
            f.write(f"{str(self.qb_vt['Turbine']['TOWERTOPRAD']):<{object_length}}{'TOWERTOPRAD':<{keyword_length}} - the tower top radius [m]\n")
            f.write(f"{str(self.qb_vt['Turbine']['TOWERBOTRAD']):<{object_length}}{'TOWERBOTRAD':<{keyword_length}} - the tower bottom radius [m]\n")
            f.write('\n')

            f.write('----------------------------------------Dynamic Stall Models--------------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['Turbine']['DYNSTALLTYPE']):<{object_length}}{'DYNSTALLTYPE':<{keyword_length}} - the dynamic stall model: 0 = none; 1 = OYE; ; 2 = IAG; 3 = GORMONT-BERG; 4 = ATEFLAP\n")
            f.write(f"{str(self.qb_vt['Turbine']['TF_OYE']):<{object_length}}{'TF_OYE':<{keyword_length}} - Tf constant for the OYE dynamic stall model\n")
            f.write(f"{str(self.qb_vt['Turbine']['AM_GB']):<{object_length}}{'AM_GB':<{keyword_length}} - Am constant for the GORMONT-BERG dynamic stall model\n")
            f.write(f"{str(self.qb_vt['Turbine']['TF_ATE']):<{object_length}}{'TF_ATE':<{keyword_length}} - Tf constant for the ATEFLAP dynamic stall model\n")
            f.write(f"{str(self.qb_vt['Turbine']['TP_ATE']):<{object_length}}{'TP_ATE':<{keyword_length}} - Tp constant for the ATEFLAP dynamic stall model\n")
            f.write(f"{'IAGPARAMS':<{object_length}}{' ':<{keyword_length}} - the parameters for the IAG DS model, order according to docs & GUI\n")
            f.write(f"{' '.join(map(str, self.qb_vt['Turbine']['IAGPARAMS']))}\n")
            f.write('\n')

            f.write('----------------------------------------Aerodynamic Models----------------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['Turbine']['UNSTEADYAERO']):<{object_length}}{'UNSTEADYAERO':<{keyword_length}} - include unsteady attached flow aerodynamics? [bool]\n")
            f.write(f"{str(self.qb_vt['Turbine']['2PLIFTDRAG']):<{object_length}}{'2PLIFTDRAG':<{keyword_length}} - include the 2 point lift drag correction? [bool]\n")
            f.write(f"{str(self.qb_vt['Turbine']['HIMMELSKAMP']):<{object_length}}{'HIMMELSKAMP':<{keyword_length}} - include the Himmelskamp Stall delay? (HAWT only) [bool]\n")
            f.write(f"{str(self.qb_vt['Turbine']['TOWERSHADOW']):<{object_length}}{'TOWERSHADOW':<{keyword_length}} - include the tower shadow effect [bool]\n")
            f.write(f"{str(self.qb_vt['Turbine']['TOWERDRAG']):<{object_length}}{'TOWERDRAG':<{keyword_length}} - the tower drag coefficient [-] (if a Structural Model is used the tower drag is defined in the tower input file)\n")
            f.write('\n')

            f.write('----------------------------------------Wake Type----------------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['Turbine']['WAKETYPE']):<{object_length}}{'WAKETYPE':<{keyword_length}} - the wake integration type: 0 = EF; 1 = PC; 2 = PC2B\n ")
            f.write('\n')

            f.write('----------------------------------------Vortex Wake Parameters----------------------------------------------------------\n')
            f.write('Only used if WAKETYPE = 0\n')
            f.write(f"{str(self.qb_vt['Turbine']['WAKEINTTYPE']):<{object_length}}{'WAKEINTTYPE':<{keyword_length}} - the wake induction type (0 = linear, 1 = exponential)\n")
            f.write(f"{str(self.qb_vt['Turbine']['WAKEROLLUP']):<{object_length}}{'WAKEROLLUP':<{keyword_length}} - calculate wake self-induction [bool]\n")
            f.write(f"{str(self.qb_vt['Turbine']['TRAILINGVORT']):<{object_length}}{'TRAILINGVORT':<{keyword_length}} - include trailing vortex elements [bool]\n")
            f.write(f"{str(self.qb_vt['Turbine']['SHEDVORT']):<{object_length}}{'SHEDVORT':<{keyword_length}} - include shed vortex elements [bool]\n")
            f.write(f"{str(self.qb_vt['Turbine']['CONVECTIONTYPE']):<{object_length}}{'CONVECTIONTYPE':<{keyword_length}} - the wake convection type (0 = BL, 1 = HH, 2 = LOC)\n")
            f.write(f"{str(self.qb_vt['Turbine']['WAKERELAXATION']):<{object_length}}{'WAKERELAXATION':<{keyword_length}} - the wake relaxation factor [0-1]\n")
            f.write(f"{str(self.qb_vt['Turbine']['FIRSTWAKEROW']):<{object_length}}{'FIRSTWAKEROW':<{keyword_length}} - first wake row length [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['MAXWAKESIZE']):<{object_length}}{'MAXWAKESIZE':<{keyword_length}} - the maximum number of wake elements [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['MAXWAKEDIST']):<{object_length}}{'MAXWAKEDIST':<{keyword_length}} - the maxmimum wake distance from the rotor plane (normalized by dia) [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['WAKEREDUCTION']):<{object_length}}{'WAKEREDUCTION':<{keyword_length}} - the wake reduction factor [-]\n ")
            f.write(f"{str(self.qb_vt['Turbine']['WAKELENGTHTYPE']):<{object_length}}{'WAKELENGTHTYPE':<{keyword_length}} - the wake length type (0 = counted in rotor revolutions, 1 = counted in time steps)\n")
            f.write(f"{str(self.qb_vt['Turbine']['CONVERSIONLENGTH']):<{object_length}}{'CONVERSIONLENGTH':<{keyword_length}} - the wake conversion length (to particles) [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['NEARWAKELENGTH']):<{object_length}}{'NEARWAKELENGTH':<{keyword_length}} - the near wake length [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['ZONE1LENGTH']):<{object_length}}{'ZONE1LENGTH':<{keyword_length}} - the wake zone 1 length [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['ZONE2LENGTH']):<{object_length}}{'ZONE2LENGTH':<{keyword_length}} - the wake zone 2 length [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['ZONE3LENGTH']):<{object_length}}{'ZONE3LENGTH':<{keyword_length}} - the wake zone 3 length [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['ZONE1FACTOR']):<{object_length}}{'ZONE1FACTOR':<{keyword_length}} - the wake zone 1 factor (integer!) [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['ZONE2FACTOR']):<{object_length}}{'ZONE2FACTOR':<{keyword_length}} - the wake zone 2 factor (integer!) [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['ZONE3FACTOR']):<{object_length}}{'ZONE3FACTOR':<{keyword_length}} - the wake zone 3 factor (integer!) [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['ZONE1FACTOR_S']):<{object_length}}{'ZONE1FACTOR_S':<{keyword_length}} - the wake zone 1 spanwise factor (integer) [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['ZONE2FACTOR_S']):<{object_length}}{'ZONE2FACTOR_S':<{keyword_length}} - the wake zone 2 spanwise factor (integer) [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['ZONE3FACTOR_S']):<{object_length}}{'ZONE3FACTOR_S':<{keyword_length}} - the wake zone 3 spanwise factor (integer) [-]\n")
            f.write('\n')

            f.write('----------------------------------------Vortex Core Parameters----------------------------------------------------------\n')
            f.write('Only used if WAKETYPE = 0\n')
            f.write(f"{str(self.qb_vt['Turbine']['BOUNDCORERADIUS']):<{object_length}}{'BOUNDCORERADIUS':<{keyword_length}} - the fixed core radius of the bound blade vortex (fraction of local chord) [0-1]\n")
            f.write(f"{str(self.qb_vt['Turbine']['WAKECORERADIUS']):<{object_length}}{'WAKECORERADIUS':<{keyword_length}} - the intial core radius of the free wake vortex (fraction of local chord) [0-1]\n")
            f.write(f"{str(self.qb_vt['Turbine']['VORTEXVISCOSITY']):<{object_length}}{'VORTEXVISCOSITY':<{keyword_length}} - the turbulent vortex viscosity\n")
            f.write(f"{str(self.qb_vt['Turbine']['VORTEXSTRAIN']):<{object_length}}{'VORTEXSTRAIN':<{keyword_length}} - calculate vortex strain [bool]\n")
            f.write(f"{str(self.qb_vt['Turbine']['MAXSTRAIN']):<{object_length}}{'MAXSTRAIN':<{keyword_length}} - the maximum element strain, before elements are removed from the wake [-]\n")

            f.write('\n')

            f.write('----------------------------------------Gamma Iteration Parameters----------------------------------------------------------\n')
            f.write('Only used if WAKETYPE = 0\n')
            f.write(f"{str(self.qb_vt['Turbine']['GAMMARELAXATION']):<{object_length}}{'GAMMARELAXATION':<{keyword_length}} - the relaxation factor used in the gamma (circulation) iteration [0-1]\n")
            f.write(f"{str(self.qb_vt['Turbine']['GAMMAEPSILON']):<{object_length}}{'GAMMAEPSILON':<{keyword_length}} - the relative gamma (circulation) convergence criteria\n")
            f.write(f"{str(self.qb_vt['Turbine']['GAMMAITERATIONS']):<{object_length}}{'GAMMAITERATIONS':<{keyword_length}} - the maximum number of gamma (circulation) iterations (integer!) [-]\n")
            f.write('\n')

            f.write('----------------------------------------Unsteady BEM Parameters----------------------------------------------------------\n')
            f.write('Only used if WAKETYPE = 1\n')
            f.write(f"{str(self.qb_vt['Turbine']['POLARDISC']):<{object_length}}{'POLARDISC':<{keyword_length}} - the polar discretization for the unsteady BEM (integer!) [-]\n")
            f.write(f"{str(self.qb_vt['Turbine']['BEMTIPLOSS']):<{object_length}}{'BEMTIPLOSS':<{keyword_length}} - use BEM tip loss factor [bool]\n")
            f.write(f"{str(self.qb_vt['Turbine']['BEMSPEEDUP']):<{object_length}}{'BEMSPEEDUP':<{keyword_length}} - initial BEM convergence acceleration time [s]\n")
            f.write('\n')

            f.write('----------------------------------------Structural Model----------------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['Main']['MainFile'])+' ':<{object_length}}{'STRUCTURALFILE':<{keyword_length}} - the input file for the structural model (leave blank if unused)\n")
            f.write(f"{str(self.qb_vt['Turbine']['GEOMSTIFFNESS']):<{object_length}}{'GEOMSTIFFNESS':<{keyword_length}} - enable geometric stiffness [bool]\n")
            f.write(f"{str(self.qb_vt['Turbine']['AEROPANELLOADS']):<{object_length}}{'AEROPANELLOADS':<{keyword_length}} - enable distributed aero panel loads and gradients [bool]\n")
            f.write('\n')

            f.write('----------------------------------------Turbine Controller----------------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['Turbine']['CONTROLLERTYPE']):<{object_length}}{'CONTROLLERTYPE':<{keyword_length}} - the type of turbine controller 0 = none, 1 = BLADED, 2 = DTU, 3 = TUB\n")
            f.write(f"{str(self.qb_vt['Turbine']['CONTROLLERFILE']):<{object_length}}{'CONTROLLERFILE':<{keyword_length}} - the controller file name, WITHOUT file ending (.dll or .so ) - leave blank if unused\n")
            f.write(f"{str(self.qb_vt['Turbine']['DLL_InFile'])+' ':<{object_length}}{'PARAMETERFILE':<{keyword_length}} - the controller parameter file name (leave blank if unused)\n")
            f.write('\n')

    def write_sub_def(self):
        self.qb_vt['QBladeOcean']['SubFile'] = os.path.join('Structure', self.QBLADE_namingOut + '_Sub.str')
        sub_file = os.path.join(self.turbine_directory, self.qb_vt['QBladeOcean']['SubFile'])
        sub_file_path = os.path.join(self.QBLADE_namingOut, self.qb_vt['QBladeOcean']['SubFile'])
        object_length = 30 
        keyword_length = 25

        # Check if the paths are absolute and if so, convert them to be relative
        if os.path.isabs(self.qb_vt['QBladeOcean']['POT_RAD_FILE']):
            self.qb_vt['QBladeOcean']['POT_RAD_FILE'] = os.path.relpath(self.qb_vt['QBladeOcean']['POT_RAD_FILE'], os.path.dirname(sub_file))
        if os.path.isabs(self.qb_vt['QBladeOcean']['POT_EXC_FILE']):
            self.qb_vt['QBladeOcean']['POT_EXC_FILE'] = os.path.relpath(self.qb_vt['QBladeOcean']['POT_EXC_FILE'], os.path.dirname(sub_file))
        if os.path.isabs(self.qb_vt['QBladeOcean']['POT_HST_FILE']): 
            self.qb_vt['QBladeOcean']['POT_HST_FILE'] = os.path.relpath(self.qb_vt['QBladeOcean']['POT_HST_FILE'], os.path.dirname(sub_file))
            
        # TODO MARINEGROWTH | 

        with open(sub_file, 'w') as f:
            f.write('---------------------------------------- file generated with WEIS QBlade API ----------------------------------------\n')
            f.write('\n')
            f.write('---------------------- QBladeOcean Sub-structure definition file -----------------\n')
            f.write(f"{str(self.qb_vt['QBladeOcean']['WATERDEPTH']):<{object_length}}{'WATERDEPTH':<{keyword_length}} - design water depth [m]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['ISFLOATING']):<{object_length}}{'ISFLOATING':<{keyword_length}} - if the structure is fixed the joint coordinates are assigned in a coordinate system with O(0,0,0) at the mudline, for floaters O(0,0,0) is at the MSL and marks the floaters's NP\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['WATERDENSITY']):<{object_length}}{'WATERDENSITY':<{keyword_length}} -design density, used in flooded member mass calculations\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['WAVEKINEVAL_MOR']):<{object_length}}{'WAVEKINEVAL_MOR':<{keyword_length}} - 0 - local evaluation, 1 - eval at fixed ref pos, 2 - eval at lagged position\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['WAVEKINEVAL_POT']):<{object_length}}{'WAVEKINEVAL_MOR':<{keyword_length}} - 0 - local evaluation, 1 - eval at fixed ref pos, 2 - eval at lagged position\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['WAVEKINTAU']):<{object_length}}{'WAVEKINTAU':<{keyword_length}} - time constant for the lagged waveKin position evaluation\n")
            if self.qb_vt['QBladeOcean']['USEADVANCEDBUOYANCY']:
                f.write(f"{str(self.qb_vt['QBladeOcean']['ADVANCEDBUOYANCY']):<{object_length}}{'ADVANCEDBUOYANCY':<{keyword_length}} - using an advanced discretization technique (N must be a square int number) to calculate buoyancy of partially submerged members, especially usefull if \"lying\" cylinders are used to generate the draft\n")
            f.write('----------------------------------------Potential Flow Opptions----------------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['QBladeOcean']['STATICBUOYANCY']):<{object_length}}{'STATICBUOYANCY':<{keyword_length}} - static buoyancy, based on the MSL should be used when using morison member buoyancy combined with potential flow diffraction forces\n")
            if len(self.qb_vt['QBladeOcean']['POT_HST_FILE']) > 0:
                f.write(f"{str(self.qb_vt['QBladeOcean']['POT_HST_FILE']+' '):<{object_length}}{'POT_HST_FILE':<{keyword_length}} - the potential flow radiation file\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['POT_RAD_FILE']+' '):<{object_length}}{'POT_RAD_FILE':<{keyword_length}} - the potential flow radiation file\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['POT_EXC_FILE']+' '):<{object_length}}{'POT_EXC_FILE':<{keyword_length}} - the potential flow excitation file\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['USE_RADIATION']):<{object_length}}{'USE_RADIATION':<{keyword_length}} - use radiation forces [bool]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['DELTA_FREQ_RAD']):<{object_length}}{'DELTA_FREQ_RAD':<{keyword_length}} - frequency resolution for radiation forces (used during interpolation of IRF)[Hz]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['TRUNC_TIME_RAD']):<{object_length}}{'TRUNC_TIME_RAD':<{keyword_length}} - truncation time for radiation forces [s]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['USE_RAD_ADDMASS']):<{object_length}}{'USE_RAD_ADDMASS':<{keyword_length}} - use added mass matrix provided in radiation file\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['USE_EXCITATION']):<{object_length}}{'USE_EXCITATION':<{keyword_length}} - use excitation forces [bool]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['DELTA_FREQ_EXC']):<{object_length}}{'DELTA_FREQ_EXC':<{keyword_length}} - frequency resolution for excitation forces [Hz]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['DELTA_DIR_EXC']):<{object_length}}{'DELTA_DIR_EXC':<{keyword_length}} - direction resolution for excitation forces [deg]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['TRUNC_TIME_EXC']):<{object_length}}{'TRUNC_TIME_EXC':<{keyword_length}} - truncation time for excitation forces [s]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['POT_DIFF_FILE']+' '):<{object_length}}{'POT_DIFF_FILE':<{keyword_length}} - the potential flow diffraction file\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['POT_SUM_FILE']+' '):<{object_length}}{'POT_SUM_FILE':<{keyword_length}} - the potential flow sum file\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['DIFF_EVAL_TYPE']):<{object_length}}{'DIFF_EVAL_TYPE':<{keyword_length}} - evaluation type for difference frequency forces: 0-none, 1-explicit, 2-newman, 3-meandrift\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['USE_SUM_FREQS']):<{object_length}}{'USE_SUM_FREQS':<{keyword_length}} - use sum frequencies [bool]\n")
            f.write('\n')
            f.write('----------------------------------------Substructure Definition----------------------------------------------------------\n')
            f.write(f"{'TP_INTERFACE_POS':<{keyword_length}} - for a floater: defined in (x,y,z) [m] from the neutral point, which is located at MSL (0,0,0), for bottom fixed substructures: defined from seabed\n")
            f.write('X[m] Y[m] Z[m]\n')
            f.write(f"{str(' '.join(map(str, self.qb_vt['QBladeOcean']['TP_INTERFACE_POS'])))}\n")
            f.write('\n')
            f.write(f"{'REF_COG_POS':<{keyword_length}} - cog reference position, at which the mass matrix is evaluated\n")
            f.write('X[m] Y[m] Z[m]\n')
            f.write(f"{str(' '.join(map(str, self.qb_vt['QBladeOcean']['REF_COG_POS'])))}\n")
            f.write('\n')
            f.write(f"{'REF_HYDRO_POS':<{keyword_length}} - position of the TP center of mass defined in (x,y,z) [m] from the origin (0,0,0), which is also the neutral point in case of a floating substructure\n")
            f.write('X[m] Y[m] Z[m]\n')
            f.write(f"{str(' '.join(map(str, self.qb_vt['QBladeOcean']['REF_HYDRO_POS'])))}\n")
            f.write('\n')

            f.write(f"{'SUB_MASS':<{keyword_length}} - the floater mass matrix is defined at the REF_COG_POS\n")
            f.write(f"{(self.qb_vt['QBladeOcean']['SUB_MASS']):<.5e} {0.0:<.5e} {0.0:<.5e} {0.0:<.5e} {0.0:<.5e} {0.0:<.5e}\n")
            f.write(f"{0.0:<.5e} {(self.qb_vt['QBladeOcean']['SUB_MASS']):<.5e} {0.0:<.5e} {0.0:<.5e} {0.0:<.5e} {0.0:<.5e}\n")
            f.write(f"{0.0:<.5e} {0.0:<.5e} {self.qb_vt['QBladeOcean']['SUB_MASS']:<.5e} {0.0:<.5e} {0.0:<.5e} {0.0:<.5e}\n")
            f.write(f"{0.0:<.5e} {0.0:<.5e} {0.0:<.5e} {self.qb_vt['QBladeOcean']['SUB_INER'][0]:<.5e} {0.0:<.5e} {0.0:<.5e}\n")
            f.write(f"{0.0:<.5e} {0.0:<.5e} {0.0:<.5e} {0.0:<.5e} {self.qb_vt['QBladeOcean']['SUB_INER'][1]:<.5e} {0.0:<.5e}\n")
            f.write(f"{0.0:<.5e} {0.0:<.5e} {0.0:<.5e} {0.0:<.5e} {0.0:<.5e} {self.qb_vt['QBladeOcean']['SUB_INER'][2]:<.5e}\n")
            f.write('\n')

            f.write(f"{'SUB_HYDROADDEDMASS':<{keyword_length}} - the hydrodynamic added mass is defined and applied at the REF_HYDRO_POS \n")
            f.write('K(0,0)[N/m] K(1,1)[N/m] K(2,2)[N/m] K(3,3)[Nm/rad] K(4,4)[Nm/rad] K(4,4)[Nm/rad]\n')
            for j in range(6):
                ln = " ".join(['{:>10.5e}'.format(i) for i in (self.qb_vt['QBladeOcean']['SUB_HYDROADDEDMASS'][j,:])])
                ln = ln + "\n"
                f.write(ln)
            f.write('\n')
            
            f.write(f"{'SUB_HYDROSTIFFNESS':<{keyword_length}} - the hydrodynamic stiffness is defined and applied at the REF_HYDRO_POS\n")
            f.write('K(0,0)[N/m] K(1,1)[N/m] K(2,2)[N/m] K(3,3)[Nm/rad] K(4,4)[Nm/rad] K(4,4)[Nm/rad]\n')
            for j in range(6):
                ln = " ".join(['{:>10.5e}'.format(i) for i in (self.qb_vt['QBladeOcean']['SUB_HYDROSTIFFNESS'][j,:])])
                ln = ln + "\n"
                f.write(ln)
            f.write('\n')

            f.write(f"{'SUB_HYDRODAMPING':<{keyword_length}} - the hydrodynamic damping is defined and applied at the REF_HYDRO_POS\n")
            f.write('R(0,0)[Ns/m] R(1,1)[Ns/m] R(2,2)[Ns/m] R(3,3)[Nms/rad] R(4,4)[Nms/rad] R(4,4)[Nms/rad]\n')
            for j in range(6):
                ln = " ".join(['{:>10.5e}'.format(i) for i in (self.qb_vt['QBladeOcean']['SUB_HYDRODAMPING'][j,:])])
                ln = ln + "\n"
                f.write(ln)
            f.write('\n')

            f.write(f"{'SUB_HYDROQUADDAMPING':<{keyword_length}} - the hydrodynamic quadratic damping is defined and applied at the REF_HYDRO_POS\n")
            f.write('R(0,0)[Ns2/m2] R(1,1)[Ns2/m2] R(2,2)[Ns2/m2] R(3,3)[Nms/rd2] R(4,4)[Nms/rd2] R(4,4)[Nms/rd2]\n')
            for j in range(6):
                ln = " ".join(['{:>10.5e}'.format(i) for i in (self.qb_vt['QBladeOcean']['SUB_HYDROQUADDAMPING'][j,:])])
                ln = ln + "\n"
                f.write(ln)
            f.write('\n')

            f.write(f"{'SUB_HYDROCONSTFORCE':<{keyword_length}} - the constant hydrodynamic buoyancy (and other forces,moments)\n")
            f.write('F(0)[N] F(1)[N] F(2)[N] F(3)[Nm] F(4)[Nm] F(5)[Nm]\n')
            f.write(f"{str(' '.join(['{:>10.5e}'.format(i) for i in self.qb_vt['QBladeOcean']['SUB_HYDROCONSTFORCE']]))}\n")
            f.write('\n')

            f.write(f"{'MARINEGROWTH':<{keyword_length}}\n")
            f.write('Thickness Density Start (below MSL) End (below MSL)\n')
            f.write(f"{str(' '.join(['{:.3f}'.format(i) for i in self.qb_vt['QBladeOcean']['MARINEGROWTH']]))}\n")
            f.write('\n')

            f.write(f"{str(self.qb_vt['QBladeOcean']['SUB_STIFFTUNER']):<{object_length}}{'SUB_STIFFTUNER':<{keyword_length}} - stiffness of substructure tuner value\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['SUB_MASSTUNER']):<{object_length}}{'SUB_MASSTUNER':<{keyword_length}} - mass of substructure tuner value\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['SUB_BUOYANCYTUNER']):<{object_length}}{'SUB_BUOYANCYTUNER':<{keyword_length}} - buoyancy of substructure tuner value\n")
            f.write('\n') 

            f.write(f"{'SUBJOINTS':<{keyword_length}} - defined either from MSL (if isFLoating) or from seabed using the designDepth variable (if !isFLoating)\n")
            f.write('JointID    JointX     JointY     JointZ\n')
            for i in range(self.qb_vt['QBladeOcean']['NJoints']):
                ln = []
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['JointID'][i]))
                ln.append('{:<10f}'.format(self.qb_vt['QBladeOcean']['Jointxi'][i]))
                ln.append('{:<10f}'.format(self.qb_vt['QBladeOcean']['Jointyi'][i]))
                ln.append('{:<10f}'.format(self.qb_vt['QBladeOcean']['Jointzi'][i]))
                f.write(" ".join(ln) + '\n')
            f.write('\n') 

            if 'NElementsRigid' in self.qb_vt['QBladeOcean']:
                f.write('SUBELEMENTSRIGID\n')
                f.write('ElemID     BMASSD     DIAMETER\n')
                for i in range(len(self.qb_vt['QBladeOcean']['NElementsRigid'])):
                    ln = []
                    ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['ElemID'][i]))
                    ln.append('{:<10f}'.format(self.qb_vt['QBladeOcean']['MASSD'][i]))
                    ln.append('{:<10f}'.format(self.qb_vt['QBladeOcean']['DIAMETER'][i]))
                    f.write(" ".join(ln) + '\n')
                f.write('\n')

            if 'NElements' in self.qb_vt['QBladeOcean']: # only write this table in case flexible members were defined
                f.write('SUBELEMENTS\n')
                f.write('ElemID     MASS_[kg/m]     Eix_[N.m^2]   EA_[N]   GJ_[N.m^2]   STRPIT_[deg]    KSX_[-] KSY_[-]   RGX_[-]  RGY_[-]  XCM_[-]  YCM_[-]  YCE_[-]  XCS_[-]  YCS_[-]  DIA_[m]  DAMP[-]\n')
                for i in range(len(self.qb_vt['QBladeOcean']['NElements'])):
                    ln = []
                    ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubElemElemID'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemMASSD'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemEIx'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemEIy'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemEA'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemGJ'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemGA'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemSTRPIT'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemKSX'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemKSY'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemRGX'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemRGY'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemXCM'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemYCM'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemXCE'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemYCE'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemXCS'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemYCS'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemDIA'][i]))
                    ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['SubElemDAMP'][i]))
                    f.write(" ".join(ln) + '\n')
                f.write('\n')

            f.write('SUBCONSTRAINTS\n')
            f.write('ID         JntID      Jnt2ID     TP         Ground     Spring     DoF_tX     DoF_tY     DoF_tZ     DoF_rX     DoF_rY     DoF_rZ\n')
            for i in range(self.qb_vt['QBladeOcean']['NSubConstr']):
                ln = []
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubConstr_ID'][i]))
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubConstr_JntID'][i]))
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubConstr_Jnt2ID'][i]))
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubConstr_TP'][i]))
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubConstr_Ground'][i]))
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubConstr_Spring'][i]))
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubConstr_DoF_tX'][i]))
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubConstr_DoF_tY'][i]))
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubConstr_DoF_tZ'][i]))
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubConstr_DoF_rX'][i]))
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubConstr_DoF_rY'][i]))
                ln.append('{:<10d}'.format(self.qb_vt['QBladeOcean']['SubConstr_DoF_rZ'][i]))
                f.write(" ".join(ln) + '\n')
            f.write('\n')

            f.write('SUBMEMBERS\n')
            f.write('MemID   Jnt1ID  Jnt2ID  ElmID   ElmRot  HyCoID  IsBuoy  MaGrID  FldArea ElmDsc Name (optional)\n')
            for i in range(self.qb_vt['QBladeOcean']['NSubMembers']):
                ln = []
                ln.append('{:<7d}'.format(self.qb_vt['QBladeOcean']['MemID'][i]))
                ln.append('{:<7d}'.format(int(self.qb_vt['QBladeOcean']['Jnt1ID'][i])))
                ln.append('{:<7d}'.format(int(self.qb_vt['QBladeOcean']['Jnt2ID'][i])))
                ln.append('{:<7d}'.format(int(self.qb_vt['QBladeOcean']['ElmID'][i])))
                ln.append('{:<7d}'.format(int(self.qb_vt['QBladeOcean']['ElmRot'][i])))
                ln.append('{:<7d}'.format(int(self.qb_vt['QBladeOcean']['HyCoID'][i])))
                ln.append('{:<7d}'.format(int(self.qb_vt['QBladeOcean']['IsBuoy'][i])))
                ln.append('{:<7d}'.format(int(self.qb_vt['QBladeOcean']['MaGrID'][i])))
                ln.append('{:<7d}'.format(self.qb_vt['QBladeOcean']['FldArea'][i]))
                ln.append('{:<7d}'.format(int(self.qb_vt['QBladeOcean']['ElmDsc'][i])))
                ln.append('{}'.format(self.qb_vt['QBladeOcean']['MemberName'][i]))
                f.write(" ".join(ln) + '\n')
            f.write('\n')
            
            f.write('----------------------------------------Strip Theory Coefficients----------------------------------------------------------\n')
            f.write('HYDROMEMBERCOEFF\n')
            f.write('CoeffID CdN  CaN  CpN  MCFC\n')
            # f.write('Placeholder\n')
            for i in range(len(self.qb_vt['QBladeOcean']['HydroCdN'])):
                ln = []
                ln.append('{:<7d}'.format(self.qb_vt['QBladeOcean']['CoeffID'][i]))
                ln.append('{:.2f}'.format(self.qb_vt['QBladeOcean']['HydroCdN'][i]))
                ln.append('{:.2f}'.format(self.qb_vt['QBladeOcean']['HydroCaN'][i]))
                ln.append('{:.2f}'.format(self.qb_vt['QBladeOcean']['HydroCpN'][i]))
                ln.append('{}'.format(self.qb_vt['QBladeOcean']['MCFC'][i]))
                f.write(" ".join(ln) + '\n')
            f.write('\n')

            f.write('HYDROJOINTCOEFF\n')
            f.write('CoeffID JointID CdA CaA CpA\n')
            for i in range(len(self.qb_vt['QBladeOcean']['AxHyCoID'])):
                ln = []
                ln.append('{:<5}'.format(self.qb_vt['QBladeOcean']['AxHyCoID'][i]))
                ln.append('{:<5}'.format(self.qb_vt['QBladeOcean']['AxHyCoJnts'][i]))
                ln.append('{:<5}'.format(self.qb_vt['QBladeOcean']['CdA'][i]))
                ln.append('{:<5}'.format(self.qb_vt['QBladeOcean']['CaA'][i]))
                ln.append('{:<5}'.format(self.qb_vt['QBladeOcean']['CpA'][i]))
                f.write(" ".join(ln) + '\n')
            f.write('\n')

            f.write('----------------------------------------Mooring Definition----------------------------------------------------------\n')
            f.write('MOORELEMENTS\n')
            f.write('MooID MASS_[kg/m] EIy_[N.m^2] EA_[N]      DAMP_[-]    DIA_[m]\n')
            for i in range(self.qb_vt['QBladeOcean']['NMooLines']):
                ln = []
                ln.append('{:<5d}'.format(self.qb_vt['QBladeOcean']['MooID'][i]))
                ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['MooMass'][i]))
                ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['MooEI'][i]))
                ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['MooEA'][i]))
                ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['MooDamp']))
                ln.append('{:<.5e}'.format(self.qb_vt['QBladeOcean']['MooDiameter'][i]))
                f.write(" ".join(ln) + '\n')
            f.write('\n')

            f.write('MOORMEMBERS\n')
            f.write('ID	CONN_1			CONN_2			Len.[m]	MoorID 	HyCoID	IsBuoy	MaGrID	ElmDsc	Name\n')
            for i in range(self.qb_vt['QBladeOcean']['NMooMembers']):
                ln = []
                ln.append('{:<5d}'.format(self.qb_vt['QBladeOcean']['MooID'][i]))
                ln.append(f"{str(self.qb_vt['QBladeOcean']['CONN_1'][i]):<10}")
                ln.append(f"{str(self.qb_vt['QBladeOcean']['CONN_2'][i]):<10}")
                ln.append('{:<5f}'.format(self.qb_vt['QBladeOcean']['MooLength'][i]))
                ln.append('{:<5d}'.format(self.qb_vt['QBladeOcean']['MooID'][i]))
                ln.append('{:<5d}'.format(self.qb_vt['QBladeOcean']['MooHyCoID'][i]))
                ln.append('{:<5d}'.format(self.qb_vt['QBladeOcean']['MooIsBuoy'][i]))
                ln.append('{:<5d}'.format(self.qb_vt['QBladeOcean']['MooMaGrID'][i]))
                ln.append('{:<5d}'.format(self.qb_vt['QBladeOcean']['MooElmDsc']))
                ln.append(f"{str(self.qb_vt['QBladeOcean']['MooName'][i]):<10}")
                f.write(" ".join(ln) + '\n')
            f.write('\n')

            f.write('RGBCOLOR\n')
            f.write('R	G	B\n')
            f.write('255	200	15\n')
            f.write('\n')

            f.write('SUB_Sensor_Locations\n')
            for i in range(self.qb_vt['QBladeOcean']['NSub_Sensors']):
                ln = []
                ln.append(f"SUB_{str(self.qb_vt['QBladeOcean']['SUB_Sensors'][i])}_{str(self.qb_vt['QBladeOcean']['SUB_Sensors_RelPos'][i])}")
                f.write(" ".join(ln) + '\n')

    def write_wave_file(self):
        self.qb_vt['QBladeOcean']['lwaFile'] = os.path.join(self.QBLADE_namingOut + '.lwa')
        lwa_file = os.path.join(self.QBLADE_runDirectory, self.qb_vt['QBladeOcean']['lwaFile'])
        object_length = 30 
        keyword_length = 25

        with open(lwa_file, 'w') as f:
            f.write('---------------------------------------- file generated with WEIS QBlade API ----------------------------------------\n')
            f.write('\n')

            f.write('----------------------------------------Object Name-----------------------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['Main']['TurbineName']+'_WAVE '):<{object_length}}{'OBJECTNAME':<{keyword_length}}  - the name of the linear wave definition object\n")
            f.write('\n')

            f.write('----------------------------------------Main Parameters-------------------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['QBladeOcean']['TIMEOFFSET']):<{object_length}}{'TIMEOFFSET':<{keyword_length}}  - the time offset from t=0s [s]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['WAVETYPE']):<{object_length}}{'WAVETYPE':<{keyword_length}}  - wave type: 0=TIMESERIES, 1=COMPONENT, 2=SINGLE, 3=JONSWAP, 4=ISSC, 5=TORSETHAUGEN, 6=CUSTOM, 7=STREAMFUNCTION\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['SIGHEIGHT']):<{object_length}}{'SIGHEIGHT':<{keyword_length}}  - the significant wave height (Hs) [m]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['PEAKPERIOD']):<{object_length}}{'PEAKPERIOD':<{keyword_length}}  - the peak period (Tp) [s]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['AUTOGAMMA']):<{object_length}}{'AUTOGAMMA':<{keyword_length}}  - use gamma according to IEC (bool): 0 = OFF, 1 = ON (JONSWAP & TORSE only) [bool]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['GAMMA']):<{object_length}}{'GAMMA':<{keyword_length}}  - custom gamma (JONSWAP & TORSE only)\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['AUTOSIGMA']):<{object_length}}{'AUTOSIGMA':<{keyword_length}}  - use sigmas according to IEC (JONSWAP & TORSE only) [bool]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['SIGMA1']):<{object_length}}{'SIGMA1':<{keyword_length}}  - sigma1 (JONSWAP & TORSE only)\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['SIGMA2']):<{object_length}}{'SIGMA2':<{keyword_length}}  - sigma2 (JONSWAP & TORSE only)\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['DOUBLEPEAK']):<{object_length}}{'DOUBLEPEAK':<{keyword_length}}  - if true a double peak TORSETHAUGEN spectrum will be created, if false only a single peak (TORSE only)\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['AUTOORCHI']):<{object_length}}{'AUTOORCHI':<{keyword_length}}  - automatic OCHI-HUBBLE parameters from significant wave height (OCHI only) [bool]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['MODFREQ1']):<{object_length}}{'MODFREQ1':<{keyword_length}}  - modal frequency 1, must be '< modalfreq1 * 0.5' (OCHI only)\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['MODFREQ2']):<{object_length}}{'MODFREQ2':<{keyword_length}}  - modal frequency 2, should be larger than 0.096 (OCHI only)\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['SIGHEIGHT1']):<{object_length}}{'SIGHEIGHT1':<{keyword_length}}  - significant height 1, should be larger than height 2 (OCHI only)\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['SIGHEIGHT2']):<{object_length}}{'SIGHEIGHT2':<{keyword_length}}  - significant height 2 (OCHI only)\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['LAMBDA1']):<{object_length}}{'LAMBDA1':<{keyword_length}}  - peak shape 1 (OCHI only)\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['LAMBDA2']):<{object_length}}{'LAMBDA2':<{keyword_length}}  - peak shape 2 (OCHI only)\n")
            f.write('\n')

            f.write('----------------------------------------Frequency Discretization----------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['QBladeOcean']['DISCTYPE']):<{object_length}}{'DISCTYPE':<{keyword_length}}  - frequency discretization type: 0 = equal energy; 1 = equal frequency\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['AUTOFREQ']):<{object_length}}{'AUTOFREQ':<{keyword_length}}  - use automatic frequency range (f_in = 0.5*f_p, f_out = 10*f_p) [bool]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['FCUTIN']):<{object_length}}{'FCUTIN':<{keyword_length}}  - cut-in frequency\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['FCUTOUT']):<{object_length}}{'FCUTOUT':<{keyword_length}}  - cut-out frequency\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['MAXFBIN']):<{object_length}}{'MAXFBIN':<{keyword_length}}  - maximum frequency bin width [Hz]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['NUMFREQ']):<{object_length}}{'NUMFREQ':<{keyword_length}}  - the number of frequency bins\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['RANDSEED']):<{object_length}}{'RANDSEED':<{keyword_length}}  - the seed for the random phase generator range [0-65535]\n")
            f.write('\n')
            
            f.write('----------------------------------------Directional Discretization (Equal Energy)-----------------------------------\n')
            f.write(f"{str(self.qb_vt['QBladeOcean']['DIRTYPE']):<{object_length}}{'DIRTYPE':<{keyword_length}}  - the directional type, 0 = UNIDIRECTIONAL, 1 = COSINESPREAD\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['DIRMEAN']):<{object_length}}{'DIRMEAN':<{keyword_length}}  - mean wave direction [deg]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['DIRMAX']):<{object_length}}{'DIRMAX':<{keyword_length}}  - directional spread [deg]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['SPREADEXP']):<{object_length}}{'SPREADEXP':<{keyword_length}}  - the spreading exponent\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['NUMDIR']):<{object_length}}{'NUMDIR':<{keyword_length}}  - the number of directional bins\n")
            f.write('\n')

            f.write('----------------------------------------Embedded Constrained Wave --------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['QBladeOcean']['EMBEDWAVE']):<{object_length}}{'EMBEDWAVE':<{keyword_length}}  - add a constrained wave [bool]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['EMBEDELEV']):<{object_length}}{'EMBEDELEV':<{keyword_length}}  - the wave elevation of the embedded wave [m]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['EMBEDTIME']):<{object_length}}{'EMBEDTIME':<{keyword_length}}  - the time at which the embedded wave occurs [s]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['EMBEDXPOS']):<{object_length}}{'EMBEDXPOS':<{keyword_length}}  - the x-position at which the embedded wave occurs [m]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['EMBEDYPOS']):<{object_length}}{'EMBEDYPOS':<{keyword_length}}  - the y-position at which the embedded wave occurs [m]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['PASTESTREAM']):<{object_length}}{'PASTESTREAM':<{keyword_length}}  - paste a streamfunction wave over the embedded linear wave [bool]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['SIGHEIGHTSTREAM']):<{object_length}}{'SIGHEIGHTSTREAM':<{keyword_length}}  - the significant height of the streamfunction wave [m]\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['PERIODSTREAM']):<{object_length}}{'PERIODSTREAM':<{keyword_length}}  - the period of the streamfunction wave [s]\n")
            f.write('\n')

    def write_controller_dir(self):
        # create "Controller" subfolder
        if not os.path.isdir(os.path.join(self.turbine_directory,'Controller')):
            try:
                os.makedirs(os.path.join(self.turbine_directory,'Controller'))
            except:
                try:
                    time.sleep(random.random())
                    if not os.path.isdir(os.path.join(self.turbine_directory,'Controller')):
                        os.makedirs(os.path.join(self.turbine_directory,'Controller'))
                except:
                    print("Error tring to make '%s'!"%os.path.join(self.turbine_directory,'Controller'))
            
    def write_DISCON_in(self):
        ####### copied from openmdao_openfast.py #######
        # Generate Bladed style Interface controller input file, intended for ROSCO https://github.com/NREL/rosco.toolbox

        # Fill controller and turbine objects for ROSCO 
        # - controller
        controller = type('', (), {})()
        
        turbine = type('', (), {})()
        turbine.Cp = type('', (), {})()
        turbine.Ct = type('', (), {})()
        turbine.Cq = type('', (), {})()
        turbine.v_rated                 = self.qb_vt['DISCON_in']['v_rated']
        turbine.Cp                      = self.qb_vt['DISCON_in']['Cp']
        turbine.Ct                      = self.qb_vt['DISCON_in']['Ct']
        turbine.Cq                      = self.qb_vt['DISCON_in']['Cq']
        turbine.Cp_table                = self.qb_vt['DISCON_in']['Cp_table']
        turbine.Ct_table                = self.qb_vt['DISCON_in']['Ct_table']
        turbine.Cq_table                = self.qb_vt['DISCON_in']['Cq_table']
        turbine.pitch_initial_rad       = self.qb_vt['DISCON_in']['Cp_pitch_initial_rad']
        turbine.TSR_initial             = self.qb_vt['DISCON_in']['Cp_TSR_initial']
        turbine.TurbineName             = 'WEIS Turbine'

        # Define DISCON infile paths
        self.qb_vt['Turbine']['DLL_InFile'] =  os.path.join('Controller', self.QBLADE_namingOut + '_DISCON.IN')
        discon_in_file = os.path.abspath(os.path.join(self.turbine_directory, self.qb_vt['Turbine']['DLL_InFile']))
        self.qb_vt['DISCON_in']['PerfFileName'] = self.QBLADE_namingOut + '_Cp_Ct_Cq.txt'
        
        # Write DISCON input files
        ROSCO_utilities.write_rotor_performance(
            turbine, 
            txt_filename=os.path.join(self.turbine_directory,  os.path.join('Controller', self.qb_vt['DISCON_in']['PerfFileName']))
            )
        
        ROSCO_utilities.write_DISCON(
            turbine,
            controller,
            param_file=discon_in_file, 
            txt_filename=self.qb_vt['DISCON_in']['PerfFileName'],
            rosco_vt=self.qb_vt['DISCON_in']
            )

    def write_turbsim_input(self):
        # create "TurbSim" subfolder
        if not os.path.isdir(os.path.join(self.QBLADE_runDirectory,'wind')):
            try:
                os.makedirs(os.path.join(self.QBLADE_runDirectory,'wind'))
            except:
                try:
                    time.sleep(random.random())
                    if not os.path.isdir(os.path.join(self.QBLADE_runDirectory,'wind')):
                        os.makedirs(os.path.join(self.QBLADE_runDirectory,'wind'))
                except:
                    print("Error tring to make '%s'!"%os.path.join(self.QBLADE_runDirectory,'wind'))

        self.qb_vt['QTurbSim']['TurbSimInp'] =  os.path.join(self.QBLADE_namingOut + '_U'+ str(self.qb_vt['QTurbSim']['URef']) + '_Seed'+ str(self.qb_vt['QTurbSim']['RandSeed1']) + '.inp')
        turbsim_file = os.path.join(self.QBLADE_runDirectory, 'wind' , self.qb_vt['QTurbSim']['TurbSimInp'])
        object_length = 30 
        keyword_length = 25

        with open(turbsim_file, 'w') as f:
            f.write('---------------------------------------- file generated with WEIS QBlade API ----------------------------------------\n')
            f.write('\n')
            f.write('---------Runtime Options-----------------------------------\n')
            f.write(f"{'False':<{object_length}}{'ECHO':<{keyword_length}} - Echo input data to <RootName>.ech (flag)\n") # Hard code for the moment
            f.write(f"{str(self.qb_vt['QTurbSim']['RandSeed1']):<{object_length}}{'RandSeed1':<{keyword_length}} - First random seed  (-2147483648 to 2147483647)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['RandSeed2']):<{object_length}}{'RandSeed2':<{keyword_length}} - Second random seed (-2147483648 to 2147483647) for intrinsic pRNG, or an alternative pRNG: 'RanLux' or 'RNSNLW'\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['WrBHHTP']):<{object_length}}{'WrBHHTP':<{keyword_length}} - Output hub-height turbulence parameters in binary form?  (Generates RootName.bin)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['WrFHHTP']):<{object_length}}{'WrFHHTP':<{keyword_length}} - Output hub-height turbulence parameters in formatted form?  (Generates RootName.dat)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['WrADHH']):<{object_length}}{'WrADHH':<{keyword_length}} - Output hub-height time-series data in AeroDyn form?  (Generates RootName.hh)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['WrADFF']):<{object_length}}{'WrADFF':<{keyword_length}} - Output full-field time-series data in TurbSim/AeroDyn form? (Generates RootName.bts)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['WrBLFF']):<{object_length}}{'WrBLFF':<{keyword_length}} - Output full-field time-series data in BLADED/AeroDyn form?  (Generates RootName.wnd)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['WrADTWR']):<{object_length}}{'WrADTWR':<{keyword_length}} - Output tower time-series data? (Generates RootName.twr)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['WrHAWCFF']):<{object_length}}{'WrHAWCFF':<{keyword_length}} - [Envision addition] Output full-field time-series data in HAWC form?  (Generates RootName-u.bin, RootName-v.bin, RootName-w.bin, RootName.hawc)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['WrFMTFF']):<{object_length}}{'WrFMTFF':<{keyword_length}} - Output full-field time-series data in formatted (readable) form?  (Generates RootName.u, RootName.v, RootName.w)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['WrACT']):<{object_length}}{'WrACT':<{keyword_length}} - Output coherent turbulence time steps in AeroDyn form? (Generates RootName.cts)\n")
            # f.write(f"{str(self.qb_vt['QTurbSim']['Clockwise']):<{object_length}}{'Clockwise':<{keyword_length}} - Clockwise rotation looking downwind? (used only for full-field binary files - not necessary for AeroDyn)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['ScaleIEC']):<{object_length}}{'ScaleIEC':<{keyword_length}} - Scale IEC turbulence models to exact target standard deviation? [0=no additional scaling; 1=use hub scale uniformly; 2=use individual scales]\n")
            f.write('\n')

            f.write('--------Turbine/Model Specifications-----------------------\n')
            f.write(f"{str(self.qb_vt['QTurbSim']['NumGrid_Z']):<{object_length}}{'NumGrid_Z':<{keyword_length}} - Vertical grid-point matrix dimension\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['NumGrid_Y']):<{object_length}}{'NumGrid_Y':<{keyword_length}} - Horizontal grid-point matrix dimension\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['TimeStep']):<{object_length}}{'TimeStep':<{keyword_length}} - Time step [seconds]\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['AnalysisTime']):<{object_length}}{'AnalysisTime':<{keyword_length}} - Length of analysis time series [seconds] (program will add time if necessary: AnalysisTime = MAX(AnalysisTime, usableTimeLabel+GridWidth/MeanHHWS) )\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['usableTimeLabel']):<{object_length}}{'usableTimeLabel':<{keyword_length}} - Usable length of output time series [seconds] (program will add GridWidth/MeanHHWS seconds)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['HubHt']):<{object_length}}{'HubHt':<{keyword_length}} - Hub height [m] (should be > 0.5*GridHeight)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['GridHeight']):<{object_length}}{'GridHeight':<{keyword_length}} - Grid height [m] \n")
            f.write(f"{str(self.qb_vt['QTurbSim']['GridWidth']):<{object_length}}{'GridWidth':<{keyword_length}} - Grid width [m] (should be >= 2*(RotorRadius+ShaftLength))\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['VFlowAng']):<{object_length}}{'VFlowAng':<{keyword_length}} - Vertical mean flow (uptilt) angle [degrees]\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['HFlowAng']):<{object_length}}{'HFlowAng':<{keyword_length}} - Horizontal mean flow (skew) angle [degrees]\n")
            f.write('\n')

            f.write('--------Meteorological Boundary Conditions-------------------\n')
            f.write(f"{str(self.qb_vt['QTurbSim']['TurbModel']):<{object_length}}{'TurbModel':<{keyword_length}} - Turbulence model ('IECKAI'=Kaimal, 'IECVKM'=von Karman, 'GP_LLJ', 'NWTCUP', 'SMOOTH', 'WF_UPW', 'WF_07D', 'WF_14D', 'TIDAL', or 'NONE')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['UserFile']):<{object_length}}{'UserFile':<{keyword_length}} - Name secondary input file for user-defined spectra or time series inputs\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['IECstandard']):<{object_length}}{'IECstandard':<{keyword_length}} - Number of IEC 61400-x standard (x=1,2, or 3 with optional 61400-1 edition number (i.e. '1-Ed2') )\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['IECturbc']):<{object_length}}{'IECturbc':<{keyword_length}} - IEC turbulence characteristic ('A', 'B', 'C' or the turbulence intensity in percent) ('KHTEST' option with NWTCUP model, not used for other models)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['IEC_WindType']):<{object_length}}{'IEC_WindType':<{keyword_length}} - IEC turbulence type ('NTM'=normal, 'xETM'=extreme turbulence, 'xEWM1'=extreme 1-year wind, 'xEWM50'=extreme 50-year wind, where x=wind turbine class 1, 2, or 3)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['ETMc']):<{object_length}}{'ETMc':<{keyword_length}} - IEC Extreme Turbulence Model 'c' parameter [m/s]\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['ProfileType']):<{object_length}}{'ProfileType':<{keyword_length}} - Wind profile type ('JET';'LOG'=logarithmic;'PL'=power law;'H2L'=Log law for TIDAL spectral model;'IEC'=PL on rotor disk, LOG elsewhere; or 'default')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['ProfileFile']):<{object_length}}{'ProfileFile':<{keyword_length}} - Name of the file that contains user-defined input profiles\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['RefHt']):<{object_length}}{'RefHt':<{keyword_length}} - Height of the reference wind speed [m]\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['URef']):<{object_length}}{'URef':<{keyword_length}} - Mean (total) wind speed at the reference height [m/s] (or 'default' for JET wind profile)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['ZJetMax']):<{object_length}}{'ZJetMax':<{keyword_length}} - Jet height [m] (used only for JET wind profile, valid 70-490 m)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['PLExp']):<{object_length}}{'PLExp':<{keyword_length}} - Power law exponent [-] (or 'default')  \n")
            f.write(f"{str(self.qb_vt['QTurbSim']['Z0']):<{object_length}}{'Z0':<{keyword_length}} - Surface roughness length [m] (or 'default')\n")
            f.write('\n')

            f.write('--------Non-IEC Meteorological Boundary Conditions------------\n')
            f.write(f"{str(self.qb_vt['QTurbSim']['Latitude']):<{object_length}}{'Latitude':<{keyword_length}} - Site latitude [degrees] (or 'default')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['RICH_NO']):<{object_length}}{'RICH_NO':<{keyword_length}} - Gradient Richardson number \n")
            f.write(f"{str(self.qb_vt['QTurbSim']['UStar']):<{object_length}}{'UStar':<{keyword_length}} - Friction or shear velocity [m/s] (or 'default')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['ZI']):<{object_length}}{'ZI':<{keyword_length}} - Mixing layer depth [m] (or 'default')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['PC_UW']):<{object_length}}{'PC_UW':<{keyword_length}} - Hub mean u'w' Reynolds stress (or 'default')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['PC_UV']):<{object_length}}{'PC_UV':<{keyword_length}} - Hub mean u'v' Reynolds stress (or 'default')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['PC_VW']):<{object_length}}{'PC_VW':<{keyword_length}} - Hub mean v'w' Reynolds stress (or 'default')\n")
            f.write('\n')

            f.write('--------Spatial Coherence Parameters----------------------------\n')
            f.write(f"{str(self.qb_vt['QTurbSim']['SCMod1']):<{object_length}}{'SCMod1':<{keyword_length}} - u-component coherence model ('GENERAL','IEC','API','NONE', or 'default')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['SCMod2']):<{object_length}}{'SCMod2':<{keyword_length}} - v-component coherence model ('GENERAL','IEC','API','NONE', or 'default')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['SCMod3']):<{object_length}}{'SCMod3':<{keyword_length}} - w-component coherence model ('GENERAL','IEC','API','NONE', or 'default')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['InCDec1']):<{object_length}}{'InCDec1':<{keyword_length}} - u-component coherence parameters [-, m^-1] ('a b' in quotes or 'default')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['InCDec2']):<{object_length}}{'InCDec2':<{keyword_length}} - v-component coherence parameters [-, m^-1] ('a b' in quotes or 'default')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['InCDec3']):<{object_length}}{'InCDec3':<{keyword_length}} - w-component coherence parameters [-, m^-1] ('a b' in quotes or 'default')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['CohExp']):<{object_length}}{'CohExp':<{keyword_length}} - Coherence exponent for general model [-] (or 'default')\n")
            f.write('\n')

            f.write('--------Coherent Turbulence Scaling Parameters-------------------\n')
            f.write(f"{str(self.qb_vt['QTurbSim']['CTEventPath']):<{object_length}}{'CTEventPath':<{keyword_length}} - Name of the path where event data files are located\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['CTEventFile']):<{object_length}}{'CTEventFile':<{keyword_length}} - Type of event files ('LES', 'DNS', or 'RANDOM')\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['Randomize']):<{object_length}}{'Randomize':<{keyword_length}} - Randomize the disturbance scale and locations? (true/false)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['DistScl']):<{object_length}}{'DistScl':<{keyword_length}} - Disturbance scale (ratio of wave height to rotor disk). (Ignored when Randomize = true.)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['CTLy']):<{object_length}}{'CTLy':<{keyword_length}} - Fractional location of tower centerline from right (looking downwind) to left side of the dataset. (Ignored when Randomize = true.)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['CTLz']):<{object_length}}{'CTLz':<{keyword_length}} - Fractional location of hub height from the bottom of the dataset. (Ignored when Randomize = true.)\n")
            f.write(f"{str(self.qb_vt['QTurbSim']['CTStartTime']):<{object_length}}{'CTStartTime':<{keyword_length}} - Minimum start time for coherent structures in RootName.cts [seconds]\n")
            f.write('\n')

    def write_simulation_setup(self):
        self.qb_vt['QSim']['SimFile'] = os.path.join(self.QBLADE_namingOut + '.sim')
        sim_file = os.path.join(self.QBLADE_runDirectory, self.qb_vt['QSim']['SimFile'])
        trb_file_path = os.path.join(self.QBLADE_namingOut, self.qb_vt['Turbine']['TrbFile'])
        object_length = 30 
        keyword_length = 25

        if self.qb_vt['Turbine']['CONTROLLERTYPE'] > 0 and self.qb_vt['QSim']['PRESCRIBETYPE'] == 1:     # if controller is set but rpm has been constrained throghout the simulation
            logger.warning('WARNING: RPMPRESCRIBED has been fixed for the duration of the simulation, but a controller was included. The RPMPRESCRIBED parameter is set to 0 instead so the controller can influence the rotational speed after ramp-up is over.')
            self.qb_vt['QSim']['PRESCRIBETYPE'] = 0
            
        if not self.qb_vt['QSim']['wave_flag'] or self.qb_vt['QSim']['ISOFFSHORE'] == 0:
            self.qb_vt['QBladeOcean']['lwaFile'] = ''
        
        if not self.qb_vt['QSim']['WNDTYPE'] == 1:
            turbsim_file = ''
        elif self.qb_vt['QSim']['DLCGenerator'] and self.qb_vt['QSim']['WNDTYPE'] == 1:
            turbsim_file = self.qb_vt['QTurbSim']['TurbSimInp']
            if self.store_turbines: # manipulate path to point the store simulation to the correct wind field
                turbsim_file = os.path.join('..',turbsim_file)
        elif self.qb_vt['QSim']['WNDTYPE'] == 1:
            turbsim_file = os.path.join('wind' , self.qb_vt['QTurbSim']['TurbSimInp']).replace('.inp','.bts')
            if self.store_turbines: # manipulate path to point the store simulation to the correct wind field
                turbsim_file = os.path.join('..',turbsim_file)

        with open(sim_file, 'w') as f:
            f.write('---------------------------------------- file generated with WEIS QBlade API ----------------------------------------\n')
            f.write('\n')

            f.write('----------------------------------------Object Name-------------------------------------------------------------------\n')
            f.write(f"{str(self.QBLADE_namingOut+'_SIMULATION '):<{object_length}}{'OBJECTNAME':<{keyword_length}} - the name of the turbine object\n")
            f.write('\n')

            f.write('----------------------------------------Simulation Type----------------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['QSim']['ISOFFSHORE']):<{object_length}}{'ISOFFSHORE':<{keyword_length}} - use a number: 0 = onshore; 1 = offshore\n")
            f.write('\n')

            f.write('----------------------------------------Turbine Parameters----------------------------------------------------------\n')
            f.write('TURB_1\n') # only single turbines at this point
            f.write(f"{str(trb_file_path+' '):<{object_length}}{'TURBFILE':<{keyword_length}} - the turbine definition file(s) used in this simulation\n")
            f.write(f"{str(self.QBLADE_namingOut+'_TURBINE '):<{object_length}}{'TURBNAME':<{keyword_length}} - the (unique) name of the turbine in the simulation (results will appear under this name)\n")
            f.write(f"{str(self.qb_vt['QSim']['INITIAL_YAW']):<{object_length}}{'INITIAL_YAW':<{keyword_length}} - the initial turbine yaw in [deg]\n")
            f.write(f"{str(self.qb_vt['QSim']['INITIAL_PITCH']):<{object_length}}{'INITIAL_PITCH':<{keyword_length}} - the initial collective blade pitch in [deg]\n")
            f.write(f"{str(self.qb_vt['QSim']['INITIAL_AZIMUTH']):<{object_length}}{'INITIAL_AZIMUTH':<{keyword_length}} - the initial azimuthal rotor angle in [deg]\n")
            f.write(f"{str(self.qb_vt['QSim']['STRSUBSTEP']):<{object_length}}{'STRSUBSTEP':<{keyword_length}} - the number of structural substeps per timestep (usually 1)\n")
            f.write(f"{str(self.qb_vt['QSim']['RELAXSTEPS']):<{object_length}}{'RELAXSTEPS':<{keyword_length}} - the number of initial static structural relaxation steps\n")
            f.write(f"{str(self.qb_vt['QSim']['PRESCRIBETYPE']):<{object_length}}{'PRESCRIBETYPE':<{keyword_length}} - rotor RPM prescribe type (0 = ramp-up; 1 = whole sim; 2 = no RPM prescibed) \n")
            f.write(f"{str(self.qb_vt['QSim']['RPMPRESCRIBED']):<{object_length}}{'RPMPRESCRIBED':<{keyword_length}} - the prescribed rotor RPM [-]\n")
            f.write(f"{str(self.qb_vt['QSim']['STRITERATIONS']):<{object_length}}{'STRITERATIONS':<{keyword_length}} - number of iterations for the time integration (used when integrator is HHT or Euler)\n")
            f.write(f"{str(self.qb_vt['QSim']['MODNEWTONITER']):<{object_length}}{'MODNEWTONITER':<{keyword_length}} - use the modified newton iteration?\n")
            f.write(f"{str(self.qb_vt['QSim']['INCLUDEAERO']):<{object_length}}{'INCLUDEAERO':<{keyword_length}} - include aerodynamic forces?\n")
            f.write(f"{str(self.qb_vt['QSim']['INCLUDEHYDRO']):<{object_length}}{'INCLUDEHYDRO':<{keyword_length}} - include hydrodynamic forces?\n")
            f.write(f"{str(self.qb_vt['QSim']['GLOBPOS_X']):<{object_length}}{'GLOBPOS_X':<{keyword_length}} - the global x-position of the turbine [m]\n")
            f.write(f"{str(self.qb_vt['QSim']['GLOBPOS_Y']):<{object_length}}{'GLOBPOS_Y':<{keyword_length}} - the global y-position of the turbine [m]\n")
            f.write(f"{str(self.qb_vt['QSim']['GLOBPOS_Z']):<{object_length}}{'GLOBPOS_Z':<{keyword_length}} - the global z-position of the turbine [m]\n")
            f.write(f"{str(self.qb_vt['QSim']['GLOBROT_X']):<{object_length}}{'GLOBROT_X':<{keyword_length}} - the global rotation about the x-axis of the turbine [deg]\n")
            f.write(f"{str(self.qb_vt['QSim']['GLOBROT_Y']):<{object_length}}{'GLOBROT_Y':<{keyword_length}} - the global rotation about the y-axis of the turbine [deg]\n")
            f.write(f"{str(self.qb_vt['QSim']['GLOBROT_Z']):<{object_length}}{'GLOBROT_Z':<{keyword_length}} - the global rotation about the z-axis of the turbine [deg]\n")
            #start Dummy placeholders for the moment:
            f.write(f"{str(''):<{object_length}}{'EVENTFILE':<{keyword_length}} - the loading file name (leave blank if unused)\n")
            f.write(f"{str(''):<{object_length}}{'LOADINGFILE':<{keyword_length}} - the loading file name (leave blank if unused))\n")
            f.write(f"{str(''):<{object_length}}{'SIMFILE':<{keyword_length}} - the simulation file name (leave blank if unused)\n")
            f.write(f"{str(''):<{object_length}}{'MOTIONFILE':<{keyword_length}} - the prescribed motion file name (leave blank if unused)\n")
            #end Dummy placeholders for the moment
            f.write(f"{str(self.qb_vt['QSim']['FLOAT_SURGE']):<{object_length}}{'FLOAT_SURGE':<{keyword_length}} - the initial floater surge [m]\n")
            f.write(f"{str(self.qb_vt['QSim']['FLOAT_SWAY']):<{object_length}}{'FLOAT_SWAY':<{keyword_length}} - the initial floater sway [m]\n")
            f.write(f"{str(self.qb_vt['QSim']['FLOAT_HEAVE']):<{object_length}}{'FLOAT_HEAVE':<{keyword_length}} - the initial floater heave [m]\n")
            f.write(f"{str(self.qb_vt['QSim']['FLOAT_ROLL']):<{object_length}}{'FLOAT_ROLL':<{keyword_length}} - the initial floater roll [deg]\n")
            f.write(f"{str(self.qb_vt['QSim']['FLOAT_PITCH']):<{object_length}}{'FLOAT_PITCH':<{keyword_length}} - the initial floater pitch [deg]\n")
            f.write(f"{str(self.qb_vt['QSim']['FLOAT_YAW']):<{object_length}}{'FLOAT_YAW':<{keyword_length}} - the initial floater yaw [deg]\n")
            f.write('\n')

            f.write('----------------------------------------Simulation Settings----------------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['QSim']['TIMESTEP']):<{object_length}}{'TIMESTEP':<{keyword_length}} - the timestep size in [s]\n")
            f.write(f"{str(self.qb_vt['QSim']['NUMTIMESTEPS']):<{object_length}}{'NUMTIMESTEPS':<{keyword_length}} - the number of timesteps\n")
            f.write(f"{str(self.qb_vt['QSim']['RAMPUP']):<{object_length}}{'RAMPUP':<{keyword_length}} - the rampup time for the structural model\n")
            f.write(f"{str(self.qb_vt['QSim']['ADDDAMP']):<{object_length}}{'ADDDAMP':<{keyword_length}} - the initial time with additional damping\n")
            f.write(f"{str(self.qb_vt['QSim']['ADDDAMPFACTOR']):<{object_length}}{'ADDDAMPFACTOR':<{keyword_length}} - for the additional damping time this factor is used to increase the damping of all components\n")
            f.write(f"{str(self.qb_vt['QSim']['WAKEINTERACTION']):<{object_length}}{'WAKEINTERACTION':<{keyword_length}} - in case of multi-turbine simulation the wake interaction start at? [s]\n")
            f.write('\n')

            f.write('----------------------------------------Wind Input-----------------------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['QSim']['WNDTYPE']):<{object_length}}{'WNDTYPE':<{keyword_length}} - use a number: 0 = steady; 1 = windfield; 2 = hubheight\n")
            f.write(f"{turbsim_file+' ':<{object_length}}{'WNDNAME':<{keyword_length}} - filename of the turbsim input file, mann input file or hubheight file (with extension), leave blank if unused\n")
            f.write(f"{str(self.qb_vt['QSim']['STITCHINGTYPE']):<{object_length}}{'STITCHINGTYPE':<{keyword_length}} - the windfield stitching type; 0 = periodic; 1 = mirror\n")
            f.write(f"{str(self.qb_vt['QSim']['WINDAUTOSHIFT']):<{object_length}}{'WINDAUTOSHIFT':<{keyword_length}} - the windfield shifting automatically based on rotor diameter [bool]\n")
            f.write(f"{str(self.qb_vt['QSim']['SHIFTTIME']):<{object_length}}{'SHIFTTIME':<{keyword_length}} - the windfield is shifted by this time if WINDAUTOSHIFT = 0\n")
            f.write(f"{str(self.qb_vt['QSim']['MEANINF']):<{object_length}}{'MEANINF':<{keyword_length}} - the mean inflow velocity, overridden if a windfield or hubheight file is use\n")
            f.write(f"{str(self.qb_vt['QSim']['HORANGLE']):<{object_length}}{'HORANGLE':<{keyword_length}} - the horizontal inflow angle\n")
            f.write(f"{str(self.qb_vt['QSim']['VERTANGLE']):<{object_length}}{'VERTANGLE':<{keyword_length}} - the vertical inflow angle\n")
            f.write(f"{str(self.qb_vt['QSim']['PROFILETYPE']):<{object_length}}{'PROFILETYPE':<{keyword_length}} - the type of wind profile used (0 = Power Law; 1 = Logarithmic)\n")
            f.write(f"{str(self.qb_vt['QSim']['SHEAREXP']):<{object_length}}{'SHEAREXP':<{keyword_length}} - the shear exponent if using a power law profile, if a windfield is used these values are used to calculate the mean wake convection velocities\n")
            f.write(f"{str(self.qb_vt['QSim']['ROUGHLENGTH']):<{object_length}}{'ROUGHLENGTH':<{keyword_length}} - the roughness length if using a log profile, if a windfield is used these values are used to calculate the mean wake convection velocities\n")
            f.write(f"{str(self.qb_vt['QSim']['DIRSHEAR']):<{object_length}}{'DIRSHEAR':<{keyword_length}} - a value for the directional shear in deg/m\n")
            f.write(f"{str(self.qb_vt['QSim']['REFHEIGHT']):<{object_length}}{'REFHEIGHT':<{keyword_length}} - the reference height, used to contruct the BL profile\n")
            f.write('\n')

            f.write('----------------------------------------Ocean Depth, Waves and Currents-------------------------------------------\n')
            f.write('the following parameters only need to be set if ISOFFSHORE = 1\n')
            f.write(f"{str(self.qb_vt['QSim']['WATERDEPTH']):<{object_length}}{'WATERDEPTH':<{keyword_length}} - the water depth\n")
            f.write(f"{str(self.qb_vt['QBladeOcean']['lwaFile']+' '):<{object_length}}{'WAVEFILE':<{keyword_length}} - the path to the wave file, leave blank if unused\n")
            f.write(f"{str(self.qb_vt['QSim']['WAVESTRETCHING']):<{object_length}}{'WAVESTRETCHING':<{keyword_length}} - the type of wavestretching, 0 = vertical, 1 = wheeler, 2 = extrapolation, 3 = none\n")
            #TODO: get stiffness and shaer from windIO definition instead hard coded
            f.write(f"{str(self.qb_vt['QSim']['SEABEDSTIFF']):<{object_length}}{'SEABEDSTIFF':<{keyword_length}} - the vertical seabed stiffness [N/m^3]\n")
            f.write(f"{str(self.qb_vt['QSim']['SEABEDDAMP']):<{object_length}}{'SEABEDDAMP':<{keyword_length}} - a damping factor for the vertical seabed stiffness evaluation, between 0 and 1 [-]\n")
            f.write(f"{str(self.qb_vt['QSim']['SEABEDSHEAR']):<{object_length}}{'SEABEDSHEAR':<{keyword_length}} - a factor for the evaluation of shear forces (friction), between 0 and 1 [-]\n")
            f.write(f"{str(self.qb_vt['QSim']['SURF_CURR_U']):<{object_length}}{'SURF_CURR_U':<{keyword_length}} - near surface current velocity [m/s]\n")
            f.write(f"{str(self.qb_vt['QSim']['SURF_CURR_DIR']):<{object_length}}{'SURF_CURR_DIR':<{keyword_length}} - near surface current direction [deg]\n")
            f.write(f"{str(self.qb_vt['QSim']['SURF_CURR_DEPTH']):<{object_length}}{'SURF_CURR_DEPTH':<{keyword_length}} - near surface current depth [m]\n")
            f.write(f"{str(self.qb_vt['QSim']['SUB_CURR_U']):<{object_length}}{'SUB_CURR_U':<{keyword_length}} - sub surface current velocity [m/s]\n")
            f.write(f"{str(self.qb_vt['QSim']['SUB_CURR_DIR']):<{object_length}}{'SUB_CURR_DIR':<{keyword_length}} - sub surface current direction [deg]\n")
            f.write(f"{str(self.qb_vt['QSim']['SUB_CURR_EXP']):<{object_length}}{'SUB_CURR_EXP':<{keyword_length}} - sub surface current exponent\n")
            f.write(f"{str(self.qb_vt['QSim']['SHORE_CURR_U']):<{object_length}}{'SHORE_CURR_U':<{keyword_length}} - near shore (constant) current velocity [m/s]\n")
            f.write(f"{str(self.qb_vt['QSim']['SHORE_CURR_DIR']):<{object_length}}{'SHORE_CURR_DIR':<{keyword_length}} - near shore (constant) current direction [deg]\n")
            f.write('\n')

            f.write('----------------------------------------Global Mooring System------------------------------------------------------\n')
            #Dummy placeholders for the moment:
            f.write(f"{str(''):<{object_length}}{'MOORINGSYSTEM':<{keyword_length}} - the path to the global mooring system file, leave blank if unused\n")
            f.write('\n')

            f.write('----------------------------------------Dynamic Wake Meandering----------------------------------------------------\n')
            #Dummy placeholders for the moment:
            f.write(f"{str(''):<{object_length}}{'DWMSUMTYPE':<{keyword_length}} - the dynamic wake meandering wake summation type: 0 = DOMINANT; 1 = QUADRATIC; 2 = LINEAR\n")
            f.write('\n')

            f.write('----------------------------------------Environmental Parameters----------------------------------------------------\n')   
            f.write(f"{str(self.qb_vt['QSim']['DENSITYAIR']):<{object_length}}{'DENSITYAIR':<{keyword_length}} - the air density [kg/m^3]\n")
            f.write(f"{str(self.qb_vt['QSim']['VISCOSITYAIR']):<{object_length}}{'VISCOSITYAIR':<{keyword_length}} - the air kinematic viscosity\n")
            f.write(f"{str(self.qb_vt['QSim']['DENSITYWATER']):<{object_length}}{'DENSITYWATER':<{keyword_length}} - the water density [kg/m^3]\n")
            f.write(f"{str(self.qb_vt['QSim']['VISCOSITYWATER']):<{object_length}}{'VISCOSITYWATER':<{keyword_length}} - the water kinematic viscosity [m^2/s]\n")
            f.write(f"{str(self.qb_vt['QSim']['GRAVITY']):<{object_length}}{'GRAVITY':<{keyword_length}} - the gravity constant [m/s^2]\n")

            f.write('----------------------------------------Output Parameters----------------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['QSim']['STOREFROM']):<{object_length}}{'STOREFROM':<{keyword_length}} - the simulation stores data from this point in time, in [s]\n")
            f.write(f"{str(self.qb_vt['QSim']['STOREREPLAY']):<{object_length}}{'STOREREPLAY':<{keyword_length}} - store a replay of the simulation (warning, large memory will be required) [bool]\n")
            f.write(f"{str(self.qb_vt['QSim']['STOREAERO']):<{object_length}}{'STOREAERO':<{keyword_length}} - should the aerodynamic data be stored [bool]\n")
            f.write(f"{str(self.qb_vt['QSim']['STOREBLADE']):<{object_length}}{'STOREBLADE':<{keyword_length}} - should the local aerodynamic blade data be stored [bool]\n")
            f.write(f"{str(self.qb_vt['QSim']['STORESTRUCT']):<{object_length}}{'STORESTRUCT':<{keyword_length}} - should the structural data be stored [bool]\n")
            f.write(f"{str(self.qb_vt['QSim']['STORESIM']):<{object_length}}{'STORESIM':<{keyword_length}} - should the simulation (performance) data be stored [bool]\n")
            f.write(f"{str(self.qb_vt['QSim']['STOREHYDRO']):<{object_length}}{'STOREHYDRO':<{keyword_length}} - should the controller data be stored [bool]\n")
            f.write(f"{str(self.qb_vt['QSim']['STORECONTROLLER']):<{object_length}}{'STORECONTROLLER':<{keyword_length}} - should the controller data be stored [bool]\n")
            f.write(f"{str(self.qb_vt['QSim']['FILTERFILE']+' '):<{object_length}}{'FILTERFILE':<{keyword_length}} - filename of the results data filter file, leave blank if unused\n")
            f.write('\n')

            f.write('----------------------------------------Modal Analysis Parameters--------------------------------------------------\n')
            f.write(f"{str(self.qb_vt['QSim']['CALCMODAL']):<{object_length}}{'CALCMODAL':<{keyword_length}} - perform a modal analysis (only single turbine simulations) [bool]\n")
            f.write(f"{str(self.qb_vt['QSim']['MINFREQ']):<{object_length}}{'MINFREQ':<{keyword_length}} - store Eigenvalues, starting with this frequency\n")
            f.write(f"{str(self.qb_vt['QSim']['DELTAFREQ']):<{object_length}}{'DELTAFREQ':<{keyword_length}} - omit Eigenvalues that are closer spaced than this value\n")
            f.write(f"{str(self.qb_vt['QSim']['NUMFREQ']):<{object_length}}{'NUMFREQ':<{keyword_length}} - set the number of Eigenmodes and Eigenvalues that will be stored\n")
            f.write('\n')
