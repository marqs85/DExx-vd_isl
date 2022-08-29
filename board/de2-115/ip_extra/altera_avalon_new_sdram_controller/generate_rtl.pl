# (C) 2001-2020 Intel Corporation. All rights reserved.
# Your use of Intel Corporation's design tools, logic functions and other 
# software and tools, and its AMPP partner logic functions, and any output 
# files from any of the foregoing (including device programming or simulation 
# files), and any associated documentation or information are expressly subject 
# to the terms and conditions of the Intel Program License Subscription 
# Agreement, Intel FPGA IP License Agreement, or other applicable 
# license agreement, including, without limitation, that your use is for the 
# sole purpose of programming logic devices manufactured by Intel and sold by 
# Intel or its authorized distributors.  Please refer to the applicable 
# agreement for further details.


# generate_rtl.pl
#
# Generates a component RTL with given configuration file.

use Getopt::Long;
use europa_all;
use em_sdram_qsys;
use embedded_ip_generate_common;
use strict;

$| = 1;     # Always flush stderr

#-------------------------------------------------------------------------------
# main code
#-------------------------------------------------------------------------------

#main()
{
    # process command-line arguements
    my $infos = process_args();
    my $project = prepare_project($infos);
    
    # prepare generation options
    my $Options = &copy_of_hash($infos);
    $Options->{system_clock_rate} = $infos->{clock_frequency};
    
    &make_sdram_controller ($project->top(), $Options);

    $project->output();

    if ($infos->{sim_model_base}) {
        # Generation test component
        my $test_module_name = $infos->{project_info}{name}."_test_component";

        $infos->{project_info}{name} = $test_module_name;

        my $test_project = prepare_project($infos);
    
        # prepare generation options
        my $test_Options = &copy_of_hash($infos);
        $test_Options->{get_top_module_name} = $test_project->get_top_module_name();
        $test_Options->{_target_module_name} = $test_project->_target_module_name();
        $test_Options->{system_clock_rate} = $infos->{clock_frequency};
        &make_sodimm ($test_project->top(), $test_Options);

        $test_project->output();
    }
    exit(0);
}

