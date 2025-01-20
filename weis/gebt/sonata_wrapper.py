
import os
import numpy as np
from openmdao.api import ExplicitComponent
from SONATA.classBlade import Blade
from SONATA.utl.beam_struct_eval import beam_struct_eval
from scipy.interpolate import interp1d


weis_dir = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))

class SONATA_WEIS(ExplicitComponent):
    
    def initialize(self):
        self.options.declare('modeling_options')
        self.options.declare('analysis_options')
        self.options.declare('wt_init')

    def setup(self):
        modeling_options = self.options["modeling_options"]
        analysis_options = self.options["analysis_options"]
        # wt_init = self.options["wt_init"]

        n_span = modeling_options['WISDEM']['RotorSE']['n_span']
        n_af = modeling_options["WISDEM"]["RotorSE"]["n_af"]
        n_af_span = modeling_options["WISDEM"]["RotorSE"]["n_af_span"]
        n_opt_spar_cap_ss = next((layer['n_opt'] for layer in analysis_options['design_variables']['blade']['structure'] if layer['layer_name'].lower() == 'spar_cap_ss'), None) # analysis_options["design_variables"]["blade"]["structure"]["spar_cap_ss"]["n_opt"]
        n_opt_spar_cap_ps = next((layer['n_opt'] for layer in analysis_options['design_variables']['blade']['structure'] if layer['layer_name'].lower() == 'spar_cap_ps'), None) # analysis_options["design_variables"]["blade"]["structure"]["spar_cap_ps"]["n_opt"]
        n_opt_te_ss = next((layer['n_opt'] for layer in analysis_options['design_variables']['blade']['structure'] if layer['layer_name'].lower() == 'te_reinforcement_ss'), None) # analysis_options["design_variables"]["blade"]["structure"]["te_ss"]["n_opt"]
        n_opt_te_ps = next((layer['n_opt'] for layer in analysis_options['design_variables']['blade']['structure'] if layer['layer_name'].lower() == 'te_reinforcement_ps'), None) # analysis_options["design_variables"]["blade"]["structure"]["te_ps"]["n_opt"]

        # Blade Aero Definition Inputs
        self.add_input('grid',                  val=np.zeros(n_span),       units='deg',    desc='non-dimensional grdi along the span of the blade')
        self.add_input('chord',                 val=np.zeros(n_span),       units='m',      desc='chord at airfoil locations')
        self.add_input('twist',                 val=np.zeros(n_span),       units='deg',    desc='twist at airfoil locations')
        self.add_input('ref_axis_blade',        val=np.zeros((n_span,3)),   units='m',      desc='2D array of the coordinates (x,y,z) of the blade reference axis, defined along blade span. The coordinate system is the one of BeamDyn: it is placed at blade root with x pointing the suction side of the blade, y pointing the trailing edge and z along the blade span. A standard configuration will have negative x values (prebend), if swept positive y values, and positive z values.')
        self.add_input('pitch_axis',            val=np.zeros(n_span),                       desc='Leading-edge positions from a reference blade axis (usually blade pitch axis). Locations are normalized by the local chord length. Positive in -x direction for airfoil-aligned coordinate system')
        self.add_input('r_thick',               val=np.zeros(n_span),                       desc='1D array of the relative thicknesses of each airfoil.')
        
        # self.add_discrete_input("airfoils_name", val=n_af * [""], desc="1D array of names of airfoils.")
        self.add_input("airfoils_position",     val=np.zeros(n_af_span), desc="1D array of the non dimensional positions of the airfoils af_used defined along blade span.")
        self.add_input(
            "s_opt_spar_cap_ss",
            val=np.zeros(n_opt_spar_cap_ss),
            desc="1D array of the non-dimensional spanwise grid defined along blade axis to optimize the blade spar cap suction side",
        )
        self.add_input(
            "s_opt_spar_cap_ps",
            val=np.zeros(n_opt_spar_cap_ps),
            desc="1D array of the non-dimensional spanwise grid defined along blade axis to optimize the blade spar cap pressure side",
        )
        self.add_input(
            "s_opt_te_ss",
            val=np.zeros(n_opt_te_ss),
            desc="1D array of the non-dimensional spanwise grid defined along blade axis to optimize the blade trailing edge suction side",
        )
        self.add_input(
            "s_opt_te_ps",
            val=np.zeros(n_opt_te_ps),
            desc="1D array of the non-dimensional spanwise grid defined along blade axis to optimize the blade trailing edge pressure side",
        )
        self.add_input(
            "spar_cap_ps_opt",
            val=np.zeros(n_opt_spar_cap_ps),
            units="m",
            desc="1D array of the optimized blade spar cap thickness on the pressure side along the blade span",
        )
        self.add_input(
            "spar_cap_ss_opt",
            val=np.zeros(n_opt_spar_cap_ss),
            units="m",
            desc="1D array of the optimized blade spar cap thickness on the suction side along the blade span",
        )
        self.add_input(
            "te_ss_opt",
            val=np.zeros(n_opt_te_ss),
            units="m",
            desc="1D array of the optimized blade trailing edge thickness on the pressure side along the blade span",
        )
        self.add_input(
            "te_ps_opt",
            val=np.zeros(n_opt_te_ps),
            units="m",
            desc="1D array of the optimized blade trailing edge thickness on the suction side along the blade span",
        )

    def compute(self, inputs, outputs):
        modeling_options = self.options["modeling_options"]
        analysis_options = self.options["analysis_options"]

        job_name= 'WEIS'

        # Mimic windIO yaml dict which is used as a Sonata input with WEIS data
        wt_init = self.options["wt_init"]
        fem_dict = {'internal_structure_2d_fem': wt_init['components']['blade']['internal_structure_2d_fem']}

        for i in range(modeling_options["WISDEM"]["RotorSE"]["n_layers"]):
            if "Spar_Cap_SS".lower() in fem_dict['internal_structure_2d_fem']['layers'][i]['name'].lower():
                fem_dict['internal_structure_2d_fem']['layers'][i]["thickness"]["grid"] = inputs["s_opt_spar_cap_ss"]
                fem_dict['internal_structure_2d_fem']['layers'][i]["thickness"]["values"] = inputs["spar_cap_ss_opt"]
            if "Spar_Cap_PS".lower() in fem_dict['internal_structure_2d_fem']['layers'][i]['name'].lower():
                fem_dict['internal_structure_2d_fem']['layers'][i]["thickness"]["grid"] = inputs["s_opt_spar_cap_ps"]
                fem_dict['internal_structure_2d_fem']['layers'][i]["thickness"]["values"] = inputs["spar_cap_ps_opt"]
            if "TE_reinforcement_SS".lower() in fem_dict['internal_structure_2d_fem']['layers'][i]['name'].lower():
                fem_dict['internal_structure_2d_fem']['layers'][i]["thickness"]["grid"] = inputs["s_opt_te_ss"]
                fem_dict['internal_structure_2d_fem']['layers'][i]["thickness"]["values"] = inputs["te_ss_opt"]
            if "TE_reinforcement_PS".lower() in fem_dict['internal_structure_2d_fem']['layers'][i]['name'].lower():
                fem_dict['internal_structure_2d_fem']['layers'][i]["thickness"]["grid"] = inputs["s_opt_te_ps"]
                fem_dict['internal_structure_2d_fem']['layers'][i]["thickness"]["values"] = inputs["te_ps_opt"]

        sonata_blade_dict = {
            "name": 
                str(modeling_options['General']['qblade_configuration']['QB_run_mod']),
            "components": {
                "blade": {
                    "outer_shape_bem": {
                        "airfoil_position": {
                            "grid": inputs["airfoils_position"].tolist(),
                            "labels": wt_init['components']['blade']['outer_shape_bem']['airfoil_position']['labels']
                        },
                        "chord": {
                            "grid": inputs["grid"].tolist(),
                            "values": inputs["chord"].tolist()
                        },
                        "twist": {
                            "grid": inputs["grid"].tolist(),
                            "values": inputs["twist"].tolist()
                        },
                        "pitch_axis": {
                            "grid": inputs["grid"].tolist(),
                            "values": inputs["pitch_axis"].tolist()
                        },
                        "reference_axis": {
                            "x": {
                                "grid": inputs["grid"].tolist(),
                                "values": inputs["ref_axis_blade"][:, 0].tolist()
                            },
                            "y": {
                                "grid": inputs["grid"].tolist(),
                                "values": inputs["ref_axis_blade"][:, 1].tolist()
                            },
                            "z": {
                                "grid": inputs["grid"].tolist(),
                                "values": inputs["ref_axis_blade"][:, 2].tolist()
                            }
                        }
                    },
                    "internal_structure_2d_fem": fem_dict["internal_structure_2d_fem"]
                }
            },
            "airfoils": wt_init["airfoils"],
            "materials": wt_init["materials"]
        }

        flag_wt_ontology        = modeling_options['SONATA']['flag_wt_ontology'] # if true, use ontology definition of wind turbines for yaml files
        flag_ref_axes_wt        = modeling_options['SONATA']['flag_ref_axes_wt'] # if true, rotate reference axes from wind definition to comply with SONATA (rotorcraft # definition)
        
        attribute_str           = modeling_options['SONATA']['attribute_str']

        # 2D cross sectional plots (blade_plot_sections)
        flag_plotTheta11        = modeling_options['SONATA']['flag_plotTheta11']      # plane orientation angle
        flag_recovery           = modeling_options['SONATA']['flag_recovery']  
        flag_plotDisplacement   = modeling_options['SONATA']['flag_plotDisplacement']      # Needs recovery flag to be activated - shows displacements from loadings in cross sectional plots
        
        # 3D plots (blade_post_3dtopo)
        flag_wf                     = modeling_options['SONATA']['flag_wf']      # plot wire-frame
        flag_lft                    = modeling_options['SONATA']['flag_lft']      # plot lofted shape of blade surface (flag_wf=True obligatory); Note: create loft with grid refinement without too many radial_stations; can also export step file of lofted shape
        flag_topo                   = modeling_options['SONATA']['flag_topo']      # plot mesh topology
        c2_axis                     = modeling_options['SONATA']['c2_axis']
        flag_DeamDyn_def_transform  = modeling_options['SONATA']['flag_DeamDyn_def_transform']               # transform from SONATA to BeamDyn coordinate system
        flag_write_BeamDyn          = modeling_options['SONATA']['flag_write_BeamDyn']                       # write BeamDyn input files for follow-up OpenFAST analysis (requires flag_DeamDyn_def_transform = True)
        
        flag_write_BeamDyn_unit_convert = modeling_options['SONATA']['flag_write_BeamDyn_unit_convert']  #'mm_to_m'     # applied only when exported to BeamDyn files

        choose_cutoff = modeling_options['SONATA']['choose_cutoff']    # 0 step, 2 round
        mesh_resolution = modeling_options['SONATA']['mesh_resolution']  
        radial_stations = modeling_options['SONATA']['radial_stations']

        # create flag dictionary
        flags_dict = {"flag_wt_ontology": flag_wt_ontology, "flag_ref_axes_wt": flag_ref_axes_wt,
                    "attribute_str": attribute_str,
                    "flag_plotDisplacement": flag_plotDisplacement, "flag_plotTheta11": flag_plotTheta11,
                    "flag_wf": flag_wf, "flag_lft": flag_lft, "flag_topo": flag_topo, "mesh_resolution": mesh_resolution,
                    "flag_recovery": flag_recovery, "c2_axis": c2_axis}

        job = Blade(name=job_name, weis_dict=sonata_blade_dict, flags=flags_dict, stations=radial_stations)  # initialize job with respective yaml input file
        
        # ===== Build & mesh segments ===== #
        job.blade_gen_section(topo_flag=True, mesh_flag = True)

        
        # Define flags
        flag_3d = modeling_options['SONATA']['flag_3d']
        flag_csv_export = modeling_options['SONATA']['flag_csv_export']                        
        
        # Update flags dictionary
        flags_dict['flag_csv_export']                   = modeling_options['SONATA']['flag_csv_export']
        flags_dict['flag_DeamDyn_def_transform']        = modeling_options['SONATA']['flag_DeamDyn_def_transform']
        flags_dict['flag_write_BeamDyn']                = modeling_options['SONATA']['flag_write_BeamDyn']
        flags_dict['flag_write_BeamDyn_unit_convert']   = modeling_options['SONATA']['flag_write_BeamDyn_unit_convert']
        
        Loads_dict = {"Forces":[1.,1.,1.],"Moments":[1.,1.,1.]}

        # Set damping for BeamDyn input file
        delta = np.array([0.03, 0.03, 0.06787]) # logarithmic decrement, natural log of the ratio of the amplitudes of any two successive peaks. 3% flap and edge, 6% torsion
        zeta = 1. / np.sqrt(1.+(2.*np.pi / delta)**2.) # damping ratio,  dimensionless measure describing how oscillations in a system decay after a disturbance
        omega = np.array([0.508286, 0.694685, 4.084712])*2*np.pi # Frequency (rad/s), flap/edge/torsion
        mu1 = 2*zeta[0]/omega[0]
        mu2 = 2*zeta[1]/omega[1]
        mu3 = 2*zeta[2]/omega[2]
        mu = np.array([mu1, mu2, mu3, mu2, mu1, mu3])
                
        if not os.path.isdir(os.path.join(weis_dir,'sonata_temp')):
            os.makedirs(os.path.join(weis_dir,'sonata_temp'))

        run_dir = os.path.join(weis_dir,'sonata_temp')
        job_str = modeling_options['General']['qblade_configuration']['QB_run_mod']
        
        beam_struct_eval(flags_dict, Loads_dict, radial_stations, job, run_dir, job_str, mu)
        
        
        
        # job.blade_plot_sections(attribute=attribute_str, plotTheta11=flag_plotTheta11, plotDisplacement=flag_plotDisplacement, savepath=run_dir)
        # if flag_3d:
        #     job.blade_post_3dtopo(flag_wf=flags_dict['flag_wf'], flag_lft=flags_dict['flag_lft'], flag_topo=flags_dict['flag_topo'])
