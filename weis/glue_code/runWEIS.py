import numpy as np
import os, sys
import openmdao.api as om
from weis.glue_code.gc_LoadInputs     import WindTurbineOntologyPythonWEIS
from wisdem.glue_code.gc_WT_InitModel import yaml2openmdao
from weis.glue_code.gc_PoseOptimization  import PoseOptimizationWEIS
from weis.glue_code.glue_code         import WindPark
from openmdao.utils.mpi import MPI
from wisdem.commonse                  import fileIO
from weis.glue_code.gc_ROSCOInputs    import assign_ROSCO_values
from weis.control.tmd                 import assign_TMD_values
from openfast_io.FileTools     import save_yaml
from wisdem.inputs.validation         import simple_types
from weis.glue_code.mpi_tools import compute_optimal_nP
from openmdao.api import CaseReader
import re

if MPI:
    from weis.glue_code.mpi_tools import map_comm_heirarchical, subprocessor_loop, subprocessor_stop

def load_cached_cases(sql_path):
    if not os.path.isfile(sql_path):
        raise FileNotFoundError(f"Could not find SQL file: {sql_path}")

    cr = CaseReader(sql_path)
    past_cases = cr.get_cases('driver', recurse=True)

    cache = []
    for case in past_cases:
        outputs = {}
        outputs.update(case.get_objectives())
        outputs.update(case.get_constraints())
        outputs.update(case.outputs)
        cache.append(outputs)

    print(f"Loaded {len(cache)} cached iterations from: {sql_path}")
    return cache


def run_weis(fname_wt_input, fname_modeling_options, fname_opt_options, 
             geometry_override=None, modeling_override=None, analysis_override=None, 
             prepMPI=False, maxnP=1):
    # Load all yaml inputs and validate (also fills in defaults)
    wt_initial = WindTurbineOntologyPythonWEIS(
        fname_wt_input,
        fname_modeling_options,
        fname_opt_options,
        modeling_override=modeling_override,
        analysis_override=analysis_override
        )
    wt_init, modeling_options, opt_options = wt_initial.get_input_data()

    # Initialize openmdao problem. If running with multiple processors in MPI, use parallel finite differencing equal to the number of cores used.
    # Otherwise, initialize the WindPark system normally. Get the rank number for parallelization. We only print output files using the root processor.
    myopt = PoseOptimizationWEIS(wt_init, modeling_options, opt_options)
    
    if MPI:
        if not prepMPI:
            nFD = modeling_options['General']['openfast_configuration']['nFD']
            nOFp = modeling_options['General']['openfast_configuration']['nOFp']
        else:
            nFD = 1
            nOFp = 0
        # Define the color map for the cores (how these are distributed between finite differencing and openfast runs)
        if opt_options['driver']['design_of_experiments']['flag']:
            nFD = MPI.COMM_WORLD.Get_size()
            nOFp = 1
            rank    = MPI.COMM_WORLD.Get_rank()
            comm_map_up = comm_map_down = {}
            for r in range(MPI.COMM_WORLD.Get_size()):
                comm_map_up[r] = [r]
            color_i = 0
        else:
            nFD = max([nFD, 1])
            comm_map_down, comm_map_up, color_map = map_comm_heirarchical(nFD, nOFp)
            rank    = MPI.COMM_WORLD.Get_rank()
            if rank < len(color_map):
                try:
                    color_i = color_map[rank]
                except IndexError:
                    raise ValueError('The number of finite differencing variables is {} and the correct number of cores were not allocated'.format(nFD))
            else:
                color_i = max(color_map) + 1
            comm_i  = MPI.COMM_WORLD.Split(color_i, 1)

    else:
        color_i = 0
        rank = 0

    # make the folder_output relative to the input, if it's a relative path
    analysis_input_dir = os.path.dirname(opt_options['fname_input_analysis'])
    opt_options['general']['folder_output'] = os.path.join(analysis_input_dir,opt_options['general']['folder_output'])

    folder_output = opt_options['general']['folder_output']
    if rank == 0 and not os.path.isdir(folder_output):
        os.makedirs(folder_output,exist_ok=True)

    if color_i == 0: # the top layer of cores enters, the others sit and wait to run openfast simulations
        # if MPI and opt_options['driver']['optimization']['flag']:
        if MPI:
            if modeling_options['OpenFAST']['flag'] or modeling_options['OpenFAST_Linear']['flag']:
                # Parallel settings for OpenFAST
                modeling_options['General']['openfast_configuration']['mpi_run'] = True
                modeling_options['General']['openfast_configuration']['mpi_comm_map_down'] = comm_map_down
                if opt_options['driver']['design_of_experiments']['flag']:
                    modeling_options['General']['openfast_configuration']['cores'] = 1
                else:
                    modeling_options['General']['openfast_configuration']['cores'] = nOFp

            # Parallel settings for OpenMDAO
            if opt_options['driver']['design_of_experiments']['flag']:
                wt_opt = om.Problem(model=WindPark(modeling_options = modeling_options, opt_options = opt_options, wt_init = wt_init), reports=False)
            else:
                wt_opt = om.Problem(model=om.Group(num_par_fd=nFD), comm=comm_i, reports=False)
                wt_opt.model.add_subsystem('comp', WindPark(modeling_options = modeling_options, opt_options = opt_options), promotes=['*'])
        else:
            # Sequential finite differencing and openfast simulations
            modeling_options['General']['openfast_configuration']['mpi_run'] = False
            modeling_options['General']['openfast_configuration']['cores']   = 1
            wt_opt = om.Problem(model=WindPark(modeling_options = modeling_options, opt_options = opt_options, wt_init = wt_init), reports=False)

            restart_file = opt_options['driver']['optimization'].get('restart_file', None)
            restart_optimization = opt_options['driver']['optimization'].get('restart_optimization', False)

            if restart_optimization:
                recorder_file = myopt.opt["recorder"]["file_name"]
                recorder_file_base, recorder_file_ext = os.path.splitext(recorder_file)
                folder_output = myopt.opt["general"]["folder_output"]

                # for user-provided files or if file was generated by a previous run, we check if the file exists
                if restart_file:
                    print(f"Resuming optimization using: {restart_file}")
                    cache = load_cached_cases(restart_file)
                    next_recorder_file = recorder_file_base + '_rs_0.sql'
                    
                elif os.path.isdir(folder_output):
                    pattern = re.compile(re.escape(recorder_file_base) + r'_rs_(\d+)\.sql$')
                    max_n = -1
                    latest_restart_file = None

                    for fname in os.listdir(folder_output):
                        match = pattern.match(fname)
                        if match:
                            n = int(match.group(1))
                            if n > max_n:
                                max_n = n
                                latest_restart_file = fname

                    if latest_restart_file:
                        print(f"Resuming optimization using: {latest_restart_file}")
                        restart_file = os.path.join(folder_output, latest_restart_file)
                        cache = load_cached_cases(restart_file)
                        next_recorder_file = recorder_file_base + f'_rs_{max_n + 1}.sql'
                    else:
                        cache = None
                        next_recorder_file = recorder_file_base + '_rs_0.sql'
                else:
                    cache = None
                    next_recorder_file = recorder_file_base + '_rs_0.sql'
                    
                opt_options['cache'] = cache  # This will be passed into openmdao_qblade.py

                myopt.opt["recorder"]["file_name"] = next_recorder_file

        # If at least one of the design variables is active, setup an optimization
        if opt_options['opt_flag']:
            wt_opt = myopt.set_driver(wt_opt)
            wt_opt = myopt.set_objective(wt_opt)
            wt_opt = myopt.set_design_variables(wt_opt, wt_init)
            wt_opt = myopt.set_constraints(wt_opt)

            if opt_options['driver']['design_of_experiments']['flag']:
                wt_opt.driver.options['debug_print'] = ['desvars','ln_cons','nl_cons','objs']
                wt_opt.driver.options['procs_per_model'] = 1
        wt_opt = myopt.set_recorders(wt_opt)
        wt_opt.driver.options['debug_print'] = ['desvars','ln_cons','nl_cons','objs','totals']

        # Setup openmdao problem
        if opt_options['opt_flag']:
            wt_opt.setup()
        else:
            # If we're not performing optimization, we don't need to allocate
            # memory for the derivative arrays.
            wt_opt.setup(derivatives=False)

        # Estimate number of design variables and parallel calls to OpenFASRT given 
        # the computational resources available. This is used to setup WEIS for an MPI run
        if prepMPI:
            nFD = 0
            for dv in wt_opt.model.list_outputs(is_design_var=True, out_stream=None):
                # dv is a tuple with (name, info)
                nFD += len(dv[1]['val'])

            # number of finite differences should be at least 1
            nFD = max([1,nFD])

            # Compute number of processors
            modeling_options = compute_optimal_nP(nFD, myopt.n_OF_runs, modeling_options, opt_options, maxnP = maxnP)

        # If WEIS is called simply to prep for an MPI call, no need to proceed and simply 
        # return the number of finite differences and OpenFAST calls, and stop
        # Otherwise, keep going assigning inputs and running the OpenMDAO model/driver
        if not prepMPI:
            # Load initial wind turbine data from wt_initial to the openmdao problem
            wt_opt = yaml2openmdao(wt_opt, modeling_options, wt_init, opt_options)
            wt_opt = assign_ROSCO_values(wt_opt, modeling_options, opt_options)
            if modeling_options['flags']['TMDs']:
                wt_opt = assign_TMD_values(wt_opt, wt_init, opt_options)

            wt_opt = myopt.set_initial(wt_opt, wt_init)
            if modeling_options['OpenFAST']['flag'] or modeling_options['QBlade']['flag']:
                wt_opt = myopt.set_initial_weis(wt_opt)

            # If the user provides values in geometry_override, they overwrite
            # whatever values have been set by the yaml files.
            # This is useful for performing black-box wrapped optimization without
            # needing to modify the yaml files.
            # Some logic is used here if the user gives a smalller size for the
            # design variable than expected to input the values into the end
            # of the array.
            # This is useful when optimizing twist, where the first few indices
            # do not need to be optimized as they correspond to a circular cross-section.
            if geometry_override is not None:
                for key in geometry_override:
                    num_values = np.array(geometry_override[key]).size
                    key_size = wt_opt[key].size
                    idx_start = key_size - num_values
                    wt_opt[key][idx_start:] = geometry_override[key]

            # Place the last design variables from a previous run into the problem.
            # This needs to occur after the above setup() and yaml2openmdao() calls
            # so these values are correctly placed in the problem.
            # wt_opt = myopt.set_restart(wt_opt)

            # Restart from a previous optimization run
            if modeling_options['General']['qblade_configuration']['restart_from_sql']:
                restart_file = os.path.join(modeling_options['General']['qblade_configuration']['restart_file'])     
                if not os.path.isfile(restart_file):
                    raise FileNotFoundError('The restart file %s does not exist. Please check the path and try again.'%restart_file)
                
                cr = CaseReader(restart_file)
                cases = list(cr.get_cases('driver'))
                if len(cases) == 0:
                    raise RuntimeError(f"No driver cases found in {restart_file}")

                last_case = cases[-1]
                for name, val in last_case.get_design_vars().items():
                    wt_opt.set_val(name, val)
            
            if 'check_totals' in opt_options['driver']['optimization']:
                if opt_options['driver']['optimization']['check_totals']:
                    wt_opt.run_model()
                    totals = wt_opt.compute_totals()

            if 'check_partials' in opt_options['driver']['optimization']:
                if opt_options['driver']['optimization']['check_partials']:
                    wt_opt.run_model()
                    checks = wt_opt.check_partials(compact_print=True)

            sys.stdout.flush()
            # Run openmdao problem
            if opt_options['opt_flag']:
                wt_opt.run_driver()
            else:
                wt_opt.run_model()

            if (not MPI) or (MPI and rank == 0):
                # Save data coming from openmdao to an output yaml file
                froot_out = os.path.join(folder_output, opt_options['general']['fname_output'])
                # Remove the fst_vt key from the dictionary and write out the modeling options
                modeling_options['General']['openfast_configuration']['fst_vt'] = {}
                if not modeling_options['OpenFAST']['from_openfast']:
                    wt_initial.write_ontology(wt_opt, froot_out)
                wt_initial.write_options(froot_out)

                # openMDAO doesn't save constraint values, so we get them from this construction
                problem_var_dict = wt_opt.list_driver_vars(
                    desvar_opts=["lower", "upper",],
                    cons_opts=["lower", "upper", "equals",],
                )
                save_yaml(folder_output, "problem_vars.yaml", simple_types(problem_var_dict))

                # Save data to numpy and matlab arrays
                fileIO.save_data(froot_out, wt_opt)

    if MPI and \
            (modeling_options['OpenFAST']['flag'] or modeling_options['OpenFAST_Linear']['flag']) and \
            (not opt_options['driver']['design_of_experiments']['flag']):
        # subprocessor ranks spin, waiting for FAST simulations to run.
        sys.stdout.flush()
        if rank in comm_map_up.keys():
            subprocessor_loop(comm_map_up)
        sys.stdout.flush()

        # close signal to subprocessors
        subprocessor_stop(comm_map_down)
        sys.stdout.flush()

    # Send each core in use to a barrier synchronization
    # Next, share WEIS outputs across all processors from rank=0 (root)
    if MPI:
        MPI.COMM_WORLD.Barrier()
        rank = MPI.COMM_WORLD.Get_rank()
        if rank != 0:
            wt_opt = None
            modeling_options = None
            opt_options = None
        
        # MPI.COMM_WORLD.bcast cannot broadcast out a full OpenMDAO problem
        # We don't need it for now, but this might become an issue if we start
        # stacking multiple WEIS calls on top of each other and we need to 
        # reuse wt_opt from one call to the next
        modeling_options = MPI.COMM_WORLD.bcast(modeling_options, root = 0)
        opt_options = MPI.COMM_WORLD.bcast(opt_options, root = 0)
        MPI.COMM_WORLD.Barrier()

    return wt_opt, modeling_options, opt_options

