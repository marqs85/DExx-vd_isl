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


package require -exact qsys 12.0
source $env(QUARTUS_ROOTDIR)/../ip/altera/sopc_builder_ip/common/embedded_ip_hwtcl_common.tcl

#-------------------------------------------------------------------------------
# module properties
#-------------------------------------------------------------------------------

set_module_property NAME {altera_avalon_new_sdram_controller}
set_module_property DISPLAY_NAME {SDRAM Controller Intel FPGA IP}
set_module_property VERSION {20.1}
set_module_property GROUP {Memory Interfaces and Controllers/SDRAM}
set_module_property AUTHOR {Intel Corporation}
set_module_property INSTANTIATE_IN_SYSTEM_MODULE true
set_module_property INTERNAL false
set_module_property HIDE_FROM_SOPC true
set_module_property HIDE_FROM_QUARTUS true
set_module_property EDITABLE true
set_module_property VALIDATION_CALLBACK validate
set_module_property ELABORATION_CALLBACK elaborate

# generation fileset

add_fileset quartus_synth QUARTUS_SYNTH sub_quartus_synth
add_fileset sim_verilog SIM_VERILOG sub_sim_verilog
add_fileset sim_vhdl SIM_VHDL sub_sim_vhdl


#-------------------------------------------------------------------------------
# module parameters
#-------------------------------------------------------------------------------

# parameters

add_parameter TAC FLOAT
set_parameter_property TAC DEFAULT_VALUE {5.5}
set_parameter_property TAC DISPLAY_NAME {Access time (t_ac)}
set_parameter_property TAC UNITS {Nanoseconds}
set_parameter_property TAC DESCRIPTION {Access time from clock edge.}
set_parameter_property TAC ALLOWED_RANGES {0.0:999.0}
set_parameter_property TAC AFFECTS_GENERATION {1}
set_parameter_property TAC HDL_PARAMETER {0}

add_parameter TRCD FLOAT
set_parameter_property TRCD DEFAULT_VALUE {20.0}
set_parameter_property TRCD DISPLAY_NAME {ACTIVE to READ or WRITE delay (t_rcd)}
set_parameter_property TRCD UNITS {Nanoseconds}
set_parameter_property TRCD DESCRIPTION {ACTIVE to READ or WRITE delay.}
set_parameter_property TRCD ALLOWED_RANGES {0.0:200.0}
set_parameter_property TRCD AFFECTS_GENERATION {1}
set_parameter_property TRCD HDL_PARAMETER {0}

add_parameter TRFC FLOAT
set_parameter_property TRFC DEFAULT_VALUE {70.0}
set_parameter_property TRFC DISPLAY_NAME {Duration of refresh command (t_rfc)}
set_parameter_property TRFC UNITS {Nanoseconds}
set_parameter_property TRFC DESCRIPTION {Auto Refresh period.}
set_parameter_property TRFC ALLOWED_RANGES {0.0:700.0}
set_parameter_property TRFC AFFECTS_GENERATION {1}
set_parameter_property TRFC HDL_PARAMETER {0}

add_parameter TRP FLOAT
set_parameter_property TRP DEFAULT_VALUE {20.0}
set_parameter_property TRP DISPLAY_NAME {Duration of precharge command (t_rp)}
set_parameter_property TRP UNITS {Nanoseconds}
set_parameter_property TRP DESCRIPTION {Precharge command period.}
set_parameter_property TRP ALLOWED_RANGES {0.0:200.0}
set_parameter_property TRP AFFECTS_GENERATION {1}
set_parameter_property TRP HDL_PARAMETER {0}

add_parameter TWR FLOAT
set_parameter_property TWR DEFAULT_VALUE {14.0}
set_parameter_property TWR DISPLAY_NAME {Write recovery time (t_wr, no auto precharge)}
set_parameter_property TWR UNITS {Nanoseconds}
set_parameter_property TWR DESCRIPTION {Write recovery if explicit precharge commands are issued.}
set_parameter_property TWR ALLOWED_RANGES {0.0:140.0}
set_parameter_property TWR AFFECTS_GENERATION {1}
set_parameter_property TWR HDL_PARAMETER {0}

add_parameter casLatency INTEGER
set_parameter_property casLatency DEFAULT_VALUE {3}
set_parameter_property casLatency DISPLAY_NAME {CAS latency cycles:}
set_parameter_property casLatency ALLOWED_RANGES {1 2 3}
set_parameter_property casLatency DESCRIPTION {Latency (in clock cycles) from a read command to data out.}
set_parameter_property casLatency AFFECTS_GENERATION {1}
set_parameter_property casLatency HDL_PARAMETER {0}

add_parameter columnWidth INTEGER
set_parameter_property columnWidth DEFAULT_VALUE {8}
set_parameter_property columnWidth DISPLAY_NAME {Column}
set_parameter_property columnWidth ALLOWED_RANGES {8:14}
set_parameter_property columnWidth DESCRIPTION {Number of column address bits.}
set_parameter_property columnWidth AFFECTS_GENERATION {1}
set_parameter_property columnWidth HDL_PARAMETER {0}

add_parameter dataWidth INTEGER
set_parameter_property dataWidth DEFAULT_VALUE {32}
set_parameter_property dataWidth DISPLAY_NAME {Bits}
set_parameter_property dataWidth ALLOWED_RANGES {8 16 32 64}
set_parameter_property dataWidth DESCRIPTION {SDRAM data bus width.}
set_parameter_property dataWidth AFFECTS_GENERATION {1}
set_parameter_property dataWidth HDL_PARAMETER {0}

add_parameter generateSimulationModel BOOLEAN
set_parameter_property generateSimulationModel DEFAULT_VALUE {false}
set_parameter_property generateSimulationModel DISPLAY_NAME {Include a functional memory model in the system testbench}
set_parameter_property generateSimulationModel DESCRIPTION {Include a functional memory model in the system testbench}
set_parameter_property generateSimulationModel AFFECTS_GENERATION {1}
set_parameter_property generateSimulationModel HDL_PARAMETER {0}

add_parameter initRefreshCommands INTEGER
set_parameter_property initRefreshCommands DEFAULT_VALUE {2}
set_parameter_property initRefreshCommands DISPLAY_NAME {Initialization refresh cycles}
set_parameter_property initRefreshCommands ALLOWED_RANGES {1:8}
set_parameter_property initRefreshCommands DESCRIPTION {Initialization refresh cycles}
set_parameter_property initRefreshCommands AFFECTS_GENERATION {1}
set_parameter_property initRefreshCommands HDL_PARAMETER {0}

# unused preset parameter: model
add_parameter model STRING
set_parameter_property model DEFAULT_VALUE {single_Micron_MT48LC4M32B2_7_chip}
set_parameter_property model DISPLAY_NAME {model}
set_parameter_property model ALLOWED_RANGES {custom Micron_MT8LSDT1664HG_module Four_SDR100_8MByte_x16_chips single_Micron_MT48LC2M32B2_7_chip single_Micron_MT48LC4M32B2_7_chip single_NEC_D4564163_A80_chip_64Mb_x_16 single_Alliance_AS4LC1M16S1_10_chip single_Alliance_AS4LC2M8S0_10_chip}
set_parameter_property model AFFECTS_GENERATION {0}
set_parameter_property model HDL_PARAMETER {0}

add_parameter numberOfBanks INTEGER
set_parameter_property numberOfBanks DEFAULT_VALUE {4}
set_parameter_property numberOfBanks DISPLAY_NAME {Banks}
set_parameter_property numberOfBanks ALLOWED_RANGES {2 4}
set_parameter_property numberOfBanks DESCRIPTION {Number of SDRAM banks.}
set_parameter_property numberOfBanks AFFECTS_GENERATION {1}
set_parameter_property numberOfBanks HDL_PARAMETER {0}

add_parameter numberOfChipSelects INTEGER
set_parameter_property numberOfChipSelects DEFAULT_VALUE {1}
set_parameter_property numberOfChipSelects DISPLAY_NAME {Chip select}
set_parameter_property numberOfChipSelects ALLOWED_RANGES {1 2 4 8}
set_parameter_property numberOfChipSelects DESCRIPTION {Number of independent chip selects in the SDRAM subsystem.}
set_parameter_property numberOfChipSelects AFFECTS_GENERATION {1}
set_parameter_property numberOfChipSelects HDL_PARAMETER {0}

add_parameter pinsSharedViaTriState BOOLEAN
set_parameter_property pinsSharedViaTriState DEFAULT_VALUE {false}
set_parameter_property pinsSharedViaTriState DISPLAY_NAME {Controller shares dq/dqm/addr I/O pins}
set_parameter_property pinsSharedViaTriState DESCRIPTION {Controller shares dq/dqm/addr I/O pins}
set_parameter_property pinsSharedViaTriState AFFECTS_GENERATION {1}
set_parameter_property pinsSharedViaTriState HDL_PARAMETER {0}
set_parameter_property pinsSharedViaTriState VISIBLE {false}

add_parameter powerUpDelay FLOAT
set_parameter_property powerUpDelay DEFAULT_VALUE {100.0}
set_parameter_property powerUpDelay DISPLAY_NAME {Delay after powerup, before initialization}
set_parameter_property powerUpDelay UNITS {Microseconds}
set_parameter_property powerUpDelay ALLOWED_RANGES {0.0:999.0}
set_parameter_property powerUpDelay DESCRIPTION {The delay from stable clock and power to SDRAM initialization.}
set_parameter_property powerUpDelay AFFECTS_GENERATION {1}
set_parameter_property powerUpDelay HDL_PARAMETER {0}

add_parameter refreshPeriod FLOAT
set_parameter_property refreshPeriod DEFAULT_VALUE {15.625}
set_parameter_property refreshPeriod DISPLAY_NAME {Issue one refresh command every}
set_parameter_property refreshPeriod UNITS {Microseconds}
set_parameter_property refreshPeriod DESCRIPTION {Auto Refresh period.}
set_parameter_property refreshPeriod ALLOWED_RANGES {0.0:156.25}
set_parameter_property refreshPeriod AFFECTS_GENERATION {1}
set_parameter_property refreshPeriod HDL_PARAMETER {0}

add_parameter rowWidth INTEGER
set_parameter_property rowWidth DEFAULT_VALUE {12}
set_parameter_property rowWidth DISPLAY_NAME {Row}
set_parameter_property rowWidth DESCRIPTION {Number of row address bits.}
set_parameter_property rowWidth ALLOWED_RANGES {11:14}
set_parameter_property rowWidth AFFECTS_GENERATION {1}
set_parameter_property rowWidth HDL_PARAMETER {0}

add_parameter masteredTristateBridgeSlave INTEGER
set_parameter_property masteredTristateBridgeSlave DISPLAY_NAME {Tristate bridge selection}
set_parameter_property masteredTristateBridgeSlave DESCRIPTION {Tristate bridge selection}
set_parameter_property masteredTristateBridgeSlave AFFECTS_GENERATION {1}
set_parameter_property masteredTristateBridgeSlave HDL_PARAMETER {0}
set_parameter_property masteredTristateBridgeSlave VISIBLE {false}

# hidden/internal parameters
add_parameter TMRD LONG
set_parameter_property TMRD DEFAULT_VALUE {3}
set_parameter_property TMRD DISPLAY_NAME {TMRD}
set_parameter_property TMRD ALLOWED_RANGES {0:65536}
set_parameter_property TMRD AFFECTS_GENERATION {1}
set_parameter_property TMRD HDL_PARAMETER {0}
set_parameter_property TMRD UNITS {cycles}

add_parameter initNOPDelay FLOAT
set_parameter_property initNOPDelay DEFAULT_VALUE {0.0}
set_parameter_property initNOPDelay DISPLAY_NAME {initNOPDelay}
set_parameter_property initNOPDelay AFFECTS_GENERATION {1}
set_parameter_property initNOPDelay HDL_PARAMETER {0}

add_parameter registerDataIn BOOLEAN
set_parameter_property registerDataIn DEFAULT_VALUE {true}
set_parameter_property registerDataIn DISPLAY_NAME {registerDataIn}
set_parameter_property registerDataIn AFFECTS_GENERATION {1}
set_parameter_property registerDataIn HDL_PARAMETER {0}

# system info parameters
add_parameter clockRate LONG
set_parameter_property clockRate DEFAULT_VALUE {50000000}
set_parameter_property clockRate DISPLAY_NAME {clockRate}
set_parameter_property clockRate AFFECTS_GENERATION {1}
set_parameter_property clockRate HDL_PARAMETER {0}
set_parameter_property clockRate SYSTEM_INFO {clock_rate clk}
set_parameter_property clockRate SYSTEM_INFO_TYPE {CLOCK_RATE}
set_parameter_property clockRate SYSTEM_INFO_ARG {clk}

add_parameter componentName STRING
set_parameter_property componentName DISPLAY_NAME {componentName}
set_parameter_property componentName VISIBLE {0}
set_parameter_property componentName AFFECTS_GENERATION {1}
set_parameter_property componentName HDL_PARAMETER {0}
set_parameter_property componentName SYSTEM_INFO {unique_id}
set_parameter_property componentName SYSTEM_INFO_TYPE {UNIQUE_ID}

# derived parameters

add_parameter size LONG
set_parameter_property size DEFAULT_VALUE {0}
set_parameter_property size DISPLAY_NAME {size}
set_parameter_property size DERIVED {1}
set_parameter_property size AFFECTS_GENERATION {1}
set_parameter_property size HDL_PARAMETER {0}

add_parameter addressWidth INTEGER
set_parameter_property addressWidth DEFAULT_VALUE {0}
set_parameter_property addressWidth DISPLAY_NAME {addressWidth}
set_parameter_property addressWidth DERIVED {1}
set_parameter_property addressWidth AFFECTS_GENERATION {1}
set_parameter_property addressWidth HDL_PARAMETER {0}

add_parameter bankWidth INTEGER
set_parameter_property bankWidth DEFAULT_VALUE {1}
set_parameter_property bankWidth DISPLAY_NAME {bankWidth}
set_parameter_property bankWidth DERIVED {1}
set_parameter_property bankWidth ALLOWED_RANGES {0:65536}
set_parameter_property bankWidth AFFECTS_GENERATION {1}
set_parameter_property bankWidth HDL_PARAMETER {0}

#add_parameter SDRAMsize STRING
#-------------------------------------------------------------------------------
# module GUI
#-------------------------------------------------------------------------------

# display group
add_display_item {} {Memory Profile} GROUP tab

# group parameter
add_display_item {Memory Profile} {Data Width} GROUP
add_display_item {Data Width} dataWidth PARAMETER

add_display_item {Memory Profile} {Architecture} GROUP
add_display_item {Architecture} numberOfChipSelects PARAMETER
add_display_item {Architecture} numberOfBanks PARAMETER

add_display_item {Memory Profile} {Address Width} GROUP
add_display_item {Address Width} rowWidth PARAMETER
add_display_item {Address Width} columnWidth PARAMETER

add_display_item {Memory Profile} {Generic Memory model (simulation only)} GROUP
add_display_item {Generic Memory model (simulation only)} generateSimulationModel PARAMETER

add_display_item {Memory Profile} sdramSizeDisplay TEXT ""

add_display_item {} {Timing} GROUP tab

add_display_item {Timing} casLatency PARAMETER
set_display_item_property casLatency DISPLAY_HINT radio
add_display_item {Timing} initRefreshCommands PARAMETER
add_display_item {Timing} refreshPeriod PARAMETER
add_display_item {Timing} powerUpDelay PARAMETER
add_display_item {Timing} TRFC PARAMETER
add_display_item {Timing} TRP PARAMETER
add_display_item {Timing} TRCD PARAMETER
add_display_item {Timing} TAC PARAMETER
add_display_item {Timing} TWR PARAMETER

set_parameter_property TMRD VISIBLE {false}
set_parameter_property initNOPDelay VISIBLE {false}
set_parameter_property registerDataIn VISIBLE {false}
set_parameter_property model VISIBLE {false}
set_parameter_property clockRate VISIBLE {false}
set_parameter_property size VISIBLE {false}
set_parameter_property addressWidth VISIBLE {false}
set_parameter_property bankWidth VISIBLE {false}

#-------------------------------------------------------------------------------
# module validation
#-------------------------------------------------------------------------------

proc proc_calculateNanoSecondPerCAS { casLatency } {
    set clockRate [ get_parameter_value clockRate ]
    if { $clockRate > 0 } {
        set clockperiod [ expr { pow(10,9)/$clockRate } ]
        return [ expr { $clockperiod * $casLatency } ]
    } else {
        send_message error "Unknown input clock frequency."
        return 0
    }
}
proc validate {} {

	# read user and system info parameter

	set TAC [ get_parameter_value TAC ]
	set TMRD [ get_parameter_value TMRD ]
	set TRCD [ get_parameter_value TRCD ]
	set TRFC [ get_parameter_value TRFC ]
	set TRP [ get_parameter_value TRP ]
	set TWR [ get_parameter_value TWR ]
	set casLatency [ get_parameter_value casLatency ]
	set clockRate [ get_parameter_value clockRate ]
	set columnWidth [ get_parameter_value columnWidth ]
	set componentName [ get_parameter_value componentName ]
	set dataWidth [ get_parameter_value dataWidth ]
	set generateSimulationModel [ proc_get_boolean_parameter generateSimulationModel ]
	set initNOPDelay [ get_parameter_value initNOPDelay ]
	set initRefreshCommands [ get_parameter_value initRefreshCommands ]
	set masteredTristateBridgeSlave [ get_parameter_value masteredTristateBridgeSlave ]
	set numberOfBanks [ get_parameter_value numberOfBanks ]
	set numberOfChipSelects [ get_parameter_value numberOfChipSelects ]
	set pinsSharedViaTriState [ proc_get_boolean_parameter pinsSharedViaTriState ]
	set powerUpDelay [ get_parameter_value powerUpDelay ]
	set refreshPeriod [ get_parameter_value refreshPeriod ]
	set registerDataIn [ proc_get_boolean_parameter registerDataIn ]
	set rowWidth [ get_parameter_value rowWidth ]
	set addressWidth [ get_parameter_value addressWidth]
	# validate parameter and update derived parameter
	set size [ get_parameter_value size ]

    ##----  Update GUI interactive  ----##
	# Calculate Address Width
	set numChipSelectBits [ log2ceil $numberOfChipSelects ]
	set numBankBits [ log2ceil $numberOfBanks ]
	set addressWidth [ expr {$rowWidth + $columnWidth + $numChipSelectBits + $numBankBits} ]
	# Calculate RAM size
	set memSizeInBits [ expr {[expr 1<<$addressWidth] * $dataWidth} ]
	set preMemSizeInBytes [ expr {$memSizeInBits/8} ]
	set size $preMemSizeInBytes

	# Update SDRAM memory size on GUI display according to user setting
	set sizeInBytes $size
	set sizeInBits [ expr {$sizeInBytes * 8} ]
	set SDRAM_TABLE "<html><table border=\"0\" width=\"100%\">
	            <tr><td valign=\"top\"><font size=3><b>Memory Size =</b></td>
	            <td valign=\"top\"><font size=3><b>[expr {$sizeInBytes/1048576}] MBytes</font></b><br><b>[expr {$sizeInBits/$dataWidth}] x $dataWidth<br>[expr {$sizeInBits/1048576}] MBits</b></td>
	            </tr></table></html>"
	set_display_item_property sdramSizeDisplay TEXT $SDRAM_TABLE


        # Enable/disable the tristate slave parameter based on the pin sharing
        if { $pinsSharedViaTriState } {
          set_parameter_property masteredTristateBridgeSlave ENABLED {true}
        } else {
          set_parameter_property masteredTristateBridgeSlave ENABLED {false}
        }

	# GUI parameter enabling and disabling
        # validate Bad Row
        if { $rowWidth < 11 || $rowWidth > 14 } {
            send_message error "Invalid row width. Row width should be between 11 and 14."
        }
        # validate Bad Col
        if { $columnWidth < 8 || $columnWidth >= $rowWidth } {
            send_message error "Invalid column width. Column width should be more than 8 and less than row width."
        }
        # validate init refresh
        if { $initRefreshCommands < 1 || $initRefreshCommands > 8 } {
            send_message error "Only integral numbers of refreshes between 1 and 8 are supported."
        }
        # validate refresh period
        if { $refreshPeriod >= 156.25 } {
            send_message error "Invalid refresh period (must be < 156.26)."
        }
        # validate power up delay
        if { $powerUpDelay >= 1000 } {
            send_message error "Invalid powerup delay (must be < 1000)."
        }
        # validate TRFC delay
        if { $TRFC >= 700 } {
            send_message error "Invalid refresh command duration t_rfc (must be < 700)."
        }
        # validate TRP delay
        if { $TRP >= 200 } {
            send_message error "Invalid precharge command period t_rp (must be < 200)."
        }
        # validate TRCD delay
        if { $TRCD >= 200 } {
            send_message error "Invalid active to read or write delay t_rcd (must be < 200)."
        }
        # validate TAC
        set nanoSecondsPerCAS [ proc_calculateNanoSecondPerCAS $casLatency ]

        if { $nanoSecondsPerCAS == 0 } {
            send_message error "Calculated CAS latency is 0 ns."
        }
        if { $TAC >= $nanoSecondsPerCAS } {
            send_message error "Invalid access time t_ac (must be < $nanoSecondsPerCAS ns)."
        }
        # validate TWR
        if { $TWR >= 140 } {
            send_message error "Invalid non-auto precharge time (must be < 140)."
        }

        send_message info "SDRAM Controller will only be supported in Quartus Prime Standard Edition in the future release."

        # embedded software assignments

	set_module_assignment embeddedsw.CMacro.CAS_LATENCY "$casLatency"
	set_module_assignment embeddedsw.CMacro.CONTENTS_INFO ""
	set_module_assignment embeddedsw.CMacro.INIT_NOP_DELAY "$initNOPDelay"
	set_module_assignment embeddedsw.CMacro.INIT_REFRESH_COMMANDS "$initRefreshCommands"
	set_module_assignment embeddedsw.CMacro.IS_INITIALIZED "1"
	set_module_assignment embeddedsw.CMacro.POWERUP_DELAY "$powerUpDelay"
	set_module_assignment embeddedsw.CMacro.REFRESH_PERIOD "$refreshPeriod"
	set_module_assignment embeddedsw.CMacro.REGISTER_DATA_IN "$registerDataIn"
	set_module_assignment embeddedsw.CMacro.SDRAM_ADDR_WIDTH "$addressWidth"
	set_module_assignment embeddedsw.CMacro.SDRAM_BANK_WIDTH "$numBankBits"
	set_module_assignment embeddedsw.CMacro.SDRAM_COL_WIDTH "$columnWidth"
	set_module_assignment embeddedsw.CMacro.SDRAM_DATA_WIDTH "$dataWidth"
	set_module_assignment embeddedsw.CMacro.SDRAM_NUM_BANKS "$numberOfBanks"
	set_module_assignment embeddedsw.CMacro.SDRAM_NUM_CHIPSELECTS "$numberOfChipSelects"
	set_module_assignment embeddedsw.CMacro.SDRAM_ROW_WIDTH "$rowWidth"
	set_module_assignment embeddedsw.CMacro.SHARED_DATA "$pinsSharedViaTriState"
	set_module_assignment embeddedsw.CMacro.SIM_MODEL_BASE "$generateSimulationModel"
	set_module_assignment embeddedsw.CMacro.STARVATION_INDICATOR "0"
	set_module_assignment embeddedsw.CMacro.T_AC "$TAC"
	set_module_assignment embeddedsw.CMacro.T_MRD "$TMRD"
	set_module_assignment embeddedsw.CMacro.T_RCD "$TRCD"
	set_module_assignment embeddedsw.CMacro.T_RFC "$TRFC"
	set_module_assignment embeddedsw.CMacro.T_RP "$TRP"
	set_module_assignment embeddedsw.CMacro.T_WR "$TWR"
	set_module_assignment embeddedsw.memoryInfo.DAT_SYM_INSTALL_DIR {SIM_DIR}
	set_module_assignment embeddedsw.memoryInfo.GENERATE_DAT_SYM {1}
	set_module_assignment embeddedsw.memoryInfo.MEM_INIT_DATA_WIDTH "$dataWidth"

	# ignore this for now
	set_module_assignment embeddedsw.CMacro.TRISTATE_BRIDGE_SLAVE {""}


	if { $generateSimulationModel } {
	    set_module_assignment testbench.partner.map.clk {my_partner.clk}
	    set_module_assignment testbench.partner.map.wire {my_partner.conduit}
	    set_module_assignment testbench.partner.my_partner.class {altera_sdram_partner_module}
	    set_module_assignment testbench.partner.my_partner.parameter.CAS_LATENCY "$casLatency"
	    set_module_assignment testbench.partner.my_partner.parameter.CONTR_NAME "$componentName"
	    set_module_assignment testbench.partner.my_partner.parameter.SDRAM_BANK_WIDTH "$numBankBits"
	    set_module_assignment testbench.partner.my_partner.parameter.SDRAM_COL_WIDTH "$columnWidth"
	    set_module_assignment testbench.partner.my_partner.parameter.SDRAM_DATA_WIDTH "$dataWidth"
	    set_module_assignment testbench.partner.my_partner.parameter.SDRAM_NUM_CHIPSELECTS "$numberOfChipSelects"
	    set_module_assignment testbench.partner.my_partner.parameter.SDRAM_ROW_WIDTH "$rowWidth"
	    # post generation assignments
	    set_module_assignment postgeneration.simulation.init_file.param_name {INIT_FILE}
	    set_module_assignment postgeneration.simulation.init_file.param_owner {wire}
	    set_module_assignment postgeneration.simulation.init_file.type {MEM_INIT}
	}

	# update derived parameter
	set_parameter_value size $size
	set_parameter_value addressWidth $addressWidth
	set_parameter_value bankWidth $numBankBits
}

#-------------------------------------------------------------------------------
# module elaboration
#-------------------------------------------------------------------------------

proc elaborate {} {

	# read parameter

	set TAC [ get_parameter_value TAC ]
	set TMRD [ get_parameter_value TMRD ]
	set TRCD [ get_parameter_value TRCD ]
	set TRFC [ get_parameter_value TRFC ]
	set TRP [ get_parameter_value TRP ]
	set TWR [ get_parameter_value TWR ]
	set casLatency [ get_parameter_value casLatency ]
	set clockRate [ get_parameter_value clockRate ]
	set columnWidth [ get_parameter_value columnWidth ]
	set componentName [ get_parameter_value componentName ]
	set dataWidth [ get_parameter_value dataWidth ]
	set generateSimulationModel [ proc_get_boolean_parameter generateSimulationModel ]
	set initNOPDelay [ get_parameter_value initNOPDelay ]
	set initRefreshCommands [ get_parameter_value initRefreshCommands ]
	set masteredTristateBridgeSlave [ get_parameter_value masteredTristateBridgeSlave ]
	set numberOfBanks [ get_parameter_value numberOfBanks ]
	set numberOfChipSelects [ get_parameter_value numberOfChipSelects ]
	set pinsSharedViaTriState [ proc_get_boolean_parameter pinsSharedViaTriState ]
	set powerUpDelay [ get_parameter_value powerUpDelay ]
	set refreshPeriod [ get_parameter_value refreshPeriod ]
	set registerDataIn [ proc_get_boolean_parameter registerDataIn ]
	set rowWidth [ get_parameter_value rowWidth ]
	set size [ get_parameter_value size ]
	set addressWidth [ get_parameter_value addressWidth ]
	set bankWidth [ get_parameter_value bankWidth ]

	# interfaces

	add_interface clk clock sink
	set_interface_property clk clockRate {0.0}
	set_interface_property clk externallyDriven {0}

	add_interface_port clk clk clk Input 1


	add_interface reset reset sink
	set_interface_property reset associatedClock {clk}
	set_interface_property reset synchronousEdges {DEASSERT}

	add_interface_port reset reset_n reset_n Input 1


	add_interface s1 avalon slave
	set_interface_property s1 addressAlignment {DYNAMIC}
	set_interface_property s1 addressGroup {0}
	set_interface_property s1 addressSpan {16777216}
	set_interface_property s1 addressUnits {WORDS}
	set_interface_property s1 alwaysBurstMaxBurst {0}
	set_interface_property s1 associatedClock {clk}
	set_interface_property s1 associatedReset {reset}
	set_interface_property s1 bitsPerSymbol {8}
	set_interface_property s1 burstOnBurstBoundariesOnly {0}
	set_interface_property s1 burstcountUnits {WORDS}
	set_interface_property s1 constantBurstBehavior {0}
	set_interface_property s1 explicitAddressSpan {0}
	set_interface_property s1 holdTime {0}
	set_interface_property s1 interleaveBursts {0}
	set_interface_property s1 isBigEndian {0}
	set_interface_property s1 isFlash {0}
	set_interface_property s1 isMemoryDevice {1}
	set_interface_property s1 isNonVolatileStorage {0}
	set_interface_property s1 linewrapBursts {0}
	set_interface_property s1 maximumPendingReadTransactions {7}
	set_interface_property s1 minimumUninterruptedRunLength {1}
	set_interface_property s1 printableDevice {0}
	set_interface_property s1 readLatency {0}
	set_interface_property s1 readWaitStates {1}
	set_interface_property s1 readWaitTime {1}
	set_interface_property s1 registerIncomingSignals {0}
	set_interface_property s1 registerOutgoingSignals {0}
	set_interface_property s1 setupTime {0}
	set_interface_property s1 timingUnits {Cycles}
	set_interface_property s1 transparentBridge {0}
	set_interface_property s1 wellBehavedWaitrequest {0}
	set_interface_property s1 writeLatency {0}
	set_interface_property s1 writeWaitStates {0}
	set_interface_property s1 writeWaitTime {0}

	set byteenable_width [ expr { $dataWidth/8 } ]
	add_interface_port s1 az_addr address Input "$addressWidth"
	add_interface_port s1 az_be_n byteenable_n Input "$byteenable_width"
	add_interface_port s1 az_cs chipselect Input 1
	add_interface_port s1 az_data writedata Input "$dataWidth"
	add_interface_port s1 az_rd_n read_n Input 1
	add_interface_port s1 az_wr_n write_n Input 1
	add_interface_port s1 za_data readdata Output "$dataWidth"
	add_interface_port s1 za_valid readdatavalid Output 1
	add_interface_port s1 za_waitrequest waitrequest Output 1

	set_interface_assignment s1 embeddedsw.configuration.isMemoryDevice {1}

	add_interface wire conduit end

	set dqm_width [ expr { $dataWidth/8 } ]
	add_interface_port wire zs_addr export Output "$rowWidth"
	add_interface_port wire zs_ba export Output "$bankWidth"
	add_interface_port wire zs_cas_n export Output 1
	add_interface_port wire zs_cke export Output 1
	add_interface_port wire zs_cs_n export Output "$numberOfChipSelects"
	add_interface_port wire zs_dq export Bidir "$dataWidth"
	add_interface_port wire zs_dqm export Output "$dqm_width"
	add_interface_port wire zs_ras_n export Output 1
	add_interface_port wire zs_we_n export Output 1


}

#-------------------------------------------------------------------------------
# module generation
#-------------------------------------------------------------------------------

# generate
proc generate {output_name output_directory rtl_ext simgen} {
	global env
	set QUARTUS_ROOTDIR         "$env(QUARTUS_ROOTDIR)"
	set component_directory     "./"
	set component_config_file   "$output_directory/${output_name}_component_configuration.pl"

	# read parameter

	set TAC [ get_parameter_value TAC ]
	set TMRD [ get_parameter_value TMRD ]
	set TRCD [ get_parameter_value TRCD ]
	set TRFC [ get_parameter_value TRFC ]
	set TRP [ get_parameter_value TRP ]
	set TWR [ get_parameter_value TWR ]
	set casLatency [ get_parameter_value casLatency ]
	set clockRate [ get_parameter_value clockRate ]
	set columnWidth [ get_parameter_value columnWidth ]
	set componentName [ get_parameter_value componentName ]
	set dataWidth [ get_parameter_value dataWidth ]
	set generateSimulationModel [ proc_get_boolean_parameter generateSimulationModel ]
	set initNOPDelay [ get_parameter_value initNOPDelay ]
	set initRefreshCommands [ get_parameter_value initRefreshCommands ]
	set masteredTristateBridgeSlave [ get_parameter_value masteredTristateBridgeSlave ]
	set numberOfBanks [ get_parameter_value numberOfBanks ]
	set numberOfChipSelects [ get_parameter_value numberOfChipSelects ]
	set pinsSharedViaTriState [ proc_get_boolean_parameter pinsSharedViaTriState ]
	set powerUpDelay [ get_parameter_value powerUpDelay ]
	set refreshPeriod [ get_parameter_value refreshPeriod ]
	set registerDataIn [ proc_get_boolean_parameter registerDataIn ]
	set rowWidth [ get_parameter_value rowWidth ]
	set size [ get_parameter_value size ]
	set addressWidth [ get_parameter_value addressWidth ]
	set bankWidth [ get_parameter_value bankWidth ]

	# prepare config file
	set component_config    [open $component_config_file "w"]

	puts $component_config "# ${output_name} Component Configuration File"
	puts $component_config "return {"

	puts $component_config "\tclock_frequency        => \"$clockRate\","
    puts $component_config "\tregister_data_in       => \"$registerDataIn\","
    puts $component_config "\tsim_model_base         => \"$generateSimulationModel\","
    puts $component_config "\tsdram_data_width       => \"$dataWidth\","
    puts $component_config "\tsdram_addr_width       => \"$rowWidth\","
    puts $component_config "\tsdram_row_width        => \"$rowWidth\","
    puts $component_config "\tsdram_col_width        => \"$columnWidth\","
    puts $component_config "\tsdram_num_chipselects  => \"$numberOfChipSelects\","
    puts $component_config "\tsdram_num_banks        => \"$numberOfBanks\","
    puts $component_config "\trefresh_period         => \"$refreshPeriod\","
    puts $component_config "\tpowerup_delay          => \"$powerUpDelay\","
    puts $component_config "\tcas_latency            => \"$casLatency\","
    puts $component_config "\tt_rfc                  => \"$TRFC\","
    puts $component_config "\tt_rp                   => \"$TRP\","
    puts $component_config "\tt_mrd                  => \"$TMRD\","
    puts $component_config "\tt_rcd                  => \"$TRCD\","
    puts $component_config "\tt_ac                   => \"$TAC\","
    puts $component_config "\tt_wr                   => \"$TWR\","
    puts $component_config "\tinit_refresh_commands  => \"$initRefreshCommands\","
    puts $component_config "\tinit_nop_delay         => \"$initNOPDelay\","
    puts $component_config "\tshared_data            => \"0\","
    puts $component_config "\tsdram_bank_width       => \"$bankWidth\","
    puts $component_config "\ttristate_bridge_slave  => \"0\","

	puts $component_config "};"
	close $component_config

	# generate rtl
	proc_generate_component_rtl  "$component_config_file" "$component_directory" "$output_name" "$output_directory" "$rtl_ext" "$simgen"
	proc_add_generated_files "$output_name" "$output_directory" "$rtl_ext" "$simgen"
}


## Add documentation links for user guide and/or release notes
add_documentation_link "User Guide" https://documentation.altera.com/#/link/sfo1400787952932/iga1401314928585
add_documentation_link "Release Notes" https://documentation.altera.com/#/link/hco1421698042087/hco1421697689300
