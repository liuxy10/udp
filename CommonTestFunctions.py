
try: 
    from .CommonTestDefinitions import *
except:
    from CommonTestDefinitions import *
import pandas as pd
import numpy as np
import struct


########## SET ##########
# Set System Input/Output Emulation
def set_input_sm_bm_ch0(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_MB_CH0, value)
def set_input_sm_bm_ch1(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_MB_CH1, value)
def set_input_sm_bm_ch2(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_MB_CH2, value)
def set_input_sm_bm_ch3(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_MB_CH3, value)
def set_input_sm_bm_ch4(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_MB_CH4, value)
def set_input_sm_bm_ch5(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_MB_CH5, value)
def set_input_sm_bm_ch6(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_MB_CH6, value)
def set_input_sm_bm_ch7(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_MB_CH7, value)
def set_input_lin_acc_x(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SHANK_LIN_ACC_X, value)
def set_input_lin_acc_y(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SHANK_LIN_ACC_Y, value)
def set_input_lin_acc_z(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SHANK_LIN_ACC_Z, value)
def set_input_grav_x(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SHANK_GRAV_X, value)
def set_input_grav_y(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SHANK_GRAV_Y, value)
def set_input_grav_z(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SHANK_GRAV_Z, value)
def set_input_ct_angle(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_CT_ANGLE, value)
def set_input_ct_field_mag(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_CT_FIELD_MAG, value)
def set_input_ct_torque(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_CT_TORQUE, value)
def set_input_knee_angle(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_KNEE_ANGLE, value)
def set_input_torque(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_TORQUE_ACTUAL, value)
def set_input_motor_current(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_MOTOR_CURRENT, value)
def set_input_motor_temperature(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_MOTOR_TEMP, value)
def set_input_mosfet_temperature(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_MOSFET_TEMP, value)
def set_input_gcs0(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_GCS_0, value)
def set_input_gcs1(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_GCS_1, value)
def set_input_gcs2(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_GCS_2, value)
def set_input_gcs3(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_GCS_3, value)
def set_input_sm_mdb_ch0(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_MDB_CH0, value)
def set_input_sm_mdb_ch1(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_MDB_CH1, value)
def set_input_sm_mdb_ch2(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_MDB_CH2, value)
def set_input_sm_mdb_ch3(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_MDB_CH3, value)
def set_input_sm_exp_ch0(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_EXP_CH0, value)
def set_input_sm_exp_ch1(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_EXP_CH1, value)
def set_input_sm_exp_ch2(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_EXP_CH2, value)
def set_input_sm_exp_ch3(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_SM_EXP_CH3, value)
def set_input_battery_temperature(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_BAT_TEMPERATURE, value)
def set_input_battery_voltage(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_BAT_VOLTAGE, value)
def set_input_battery_current(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_BAT_CURRENT, value)
def set_input_battery_remain(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_BAT_REMAINING_CAPACITY, value)
def set_input_battery_status(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_BAT_STATUS, value)
def set_input_battery_charge(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_REL_STATE_OF_CHARGE, value)
def set_input_exp_temperature(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_INPUT_EXP_TEMP, value)
def set_output_torque(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_SYS_OUTPUT_TORQUE, value)


# Set Reactive AI Emulation =
def set_phase(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_CURRENT_ACTIVE_PHASE, value)
def set_subphase(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_CURRENT_ACTIVE_SUB_PHASE, value)
def set_activity(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_CURRENT_ACTIVITY, value)
def set_torque(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_TORQUE, value)
def set_knee_velocity(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_KNEE_VELOCITY, value)
def set_knee_motor_velocity(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_KNEE_MOTOR_VELOCITY, value)
def set_thigh_angle(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_ABS_THIGH_ANGLE, value)
def set_shank_angle(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_ABS_SHANK_ANGLE, value)
def set_knee_angle(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_KNEE_ANGLE, value)
def set_knee_motor_angle(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_KNEE_MOTOR_ANGLE, value)
def set_sagital_plane_offset_angle(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_SAGITAL_PLANE_OFFSET_ANGLE, value)
def set_max_y_pos_in_window(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_MAX_Y_POS_WITHIN_TRACK_WINDOW, value)
def set_control_mode(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_CURRENT_CONTROL_MODE, value)
def set_load_cell(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_LOAD_CELL, value)
def set_cadence(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_CURRENT_CADENCE, value)
def set_toe_off_unload_point_found(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_TOE_OFF_UNLOAD_POINT_FOUND, value)
def set_ct_knee_angle(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_KNEE_ANGLE_DUE_TO_CT, value)
def set_shank_rot_velocity(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_SHANK_ROTATIONAL_VELOCITY, value)
def set_thigh_rot_velocity(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_THIGH_ROTATIONAL_VELOCITY, value)
def set_brake_reached(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_BRAKE_SUB_PHASE_REACHED, value)
def set_drift_status(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_DRIFT_STATUS_UNKNOWN, value)
def set_step_section(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_STEP_SECTION, value)
def set_toe_load(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_TOE_LOAD, value)
def set_dif_atan_grav(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_REACTIVE_AI_DIF_ATAN_GRAV, value)

# Set Other Emulation Variables
def set_user_warning(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_BASE_SYSTEM_USER_WARNING, value)
def set_controller_modulation(wireless, value):
    return  wireless.emulate_variable(Emulation.EMULATE_CONTROLLER_MODULATION_ENABLED, value)


# Set Action
def set_hw_parameter(wireless, index, value):
    wireless.action(wireless.bionics.action_pkm_set_hw_parameter, index, value)
def start_shutdown(wireless):
    wireless.action(wireless.bionics.action_device_turn_off)


# Set User Parameter
def set_user_weight(wireless, value):
    wireless.set_variable(wireless.bionics.var_leg_user_weight, value)
def set_user_thigh_length(wireless, value):
    wireless.set_variable(wireless.bionics.var_leg_user_thigh_length, value)
def set_user_knee_height(wireless, value):
    wireless.set_variable(wireless.bionics.var_leg_user_knee_height, value)
def set_user_foot_size(wireless, value):
    wireless.set_variable(wireless.bionics.var_leg_user_foot_size, value)
def set_profile_id(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_profile_id, value)
def set_vibration_intensity(wireless, value):
    wireless.set_variable(wireless.bionics.var_base_feedback_vibration_intensity, value)
def set_stance_flexion_level(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_stance_flexion_level, value)
def set_toa_torque_level(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_toa_torque_level, value)
def set_swing_flexion_angle(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_max_swing_flexion_angle_walking, value)
def set_walking_speed(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_fa_fp_walking_speed, value)
def set_step_limit(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_fa_fp_step_limit, value)
def set_sdn_torque_level(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_sdn_torque_level, value)
def set_sdn_resistance_level(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_sdn_resistance_level, value)
def set_sup_assistance_level(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_sup_assistance_level, value)
def set_sup_speed_level(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_sup_speed_level, value)
def set_dis_resistance_level(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_dis_resistance_level, value)
def set_dis_extension_speed(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_dis_ext_speed, value)
def set_ramp_resistance_level(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_ramp_resistance_level, value)
def set_gen_assistance_level(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_gen_assistance_level, value)
def set_gen_flexion_angle(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_gen_flexion_angle, value)
def set_gen_foot_placement_angle(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_gen_foot_placement_angle, value)
def set_ramp_torque_level(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_ramp_torque_level, value)
def set_exercise_mode(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_exercise_mode, value)
def set_disable_stair_ascent(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_disable_stair_ascent, value)
def set_amputation_type(wireless, value):
    wireless.set_variable(wireless.bionics.var_power_knee_amputation_type, value)
def set_calibration_status(wireless, value):
    wireless.set_variable(wireless.bionics.var_base_calibration_status, value)


########## GET ##########
# Get Variable Functions
def get_phase(wireless):
    return Phase(wireless.get_variable(wireless.bionics.var_base_gait_phase))
def get_subphase(wireless):
    return Subphase(wireless.get_variable(wireless.bionics.var_base_gait_subphase))
def get_activity(wireless):
    return Activity(wireless.get_variable(wireless.bionics.var_base_activity))
def get_control_mode(wireless):
    return ControlMode(wireless.get_variable(wireless.bionics.var_power_knee_gait_control_mode))
def get_target_angle(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_cont_target_angle)
def get_target_velocity(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_cont_target_velocity)
def get_gain_kp(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_cont_gain_kp)
def get_gain_kd(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_cont_gain_kd)
def get_gain_km(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_cont_gain_km)
def get_hw_parameter(wireless, index):
    [value] = struct.unpack('f', wireless.action(wireless.bionics.action_pkm_get_hw_parameter, index).data)
    return value
def get_control_parameters(wireless):
    return [get_gain_kp(wireless), get_gain_kd(wireless), get_gain_km(wireless), get_target_angle(wireless), get_target_velocity(wireless)]


# Get Report Functions
def get_log_report(wireless):
    log_data, _ = wireless.end_logging(wireless.bionics.device_reports_log)
    return pd.DataFrame.from_dict(log_data)
def get_biometric_stats_report(wireless):
    return wireless.device_report(wireless.bionics.device_reports_pkm_biometric_stats)
def get_application_stats_report(wireless):
    return wireless.device_report(wireless.bionics.device_reports_pkm_application_statistics)
def get_fatal_log(wireless):
    return wireless.device_report(wireless.bionics.device_reports_pkm_fatal_log)


# Set User Parameter
def get_user_weight(wireless):
    return wireless.get_variable(wireless.bionics.var_leg_user_weight)
def get_user_thigh_length(wireless):
    return wireless.get_variable(wireless.bionics.var_leg_user_thigh_length)
def get_user_knee_height(wireless):
    return wireless.get_variable(wireless.bionics.var_leg_user_knee_height)
def get_user_foot_size(wireless):
    return wireless.get_variable(wireless.bionics.var_leg_user_foot_size)
def get_profile_id(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_profile_id)
def get_vibration_intensity(wireless):
    return wireless.get_variable(wireless.bionics.var_base_feedback_vibration_intensity)
def get_stance_flexion_level(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_stance_flexion_level)
def get_toa_torque_level(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_toa_torque_level)
def get_swing_flexion_angle(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_max_swing_flexion_angle_walking)
def get_walking_speed(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_fa_fp_walking_speed)
def get_step_limit(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_fa_fp_step_limit)
def get_sdn_torque_level(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_sdn_torque_level)
def get_sdn_resistance_level(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_sdn_resistance_level)
def get_sup_assistance_level(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_sup_assistance_level)
def get_sup_speed_level(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_sup_speed_level)
def get_dis_resistance_level(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_dis_resistance_level)
def get_dis_extension_speed(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_dis_ext_speed)
def get_ramp_resistance_level(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_ramp_resistance_level)
def get_gen_assistance_level(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_gen_assistance_level)
def get_gen_flexion_angle(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_gen_flexion_angle)
def get_gen_foot_placement_angle(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_gen_foot_placement_angle)
def get_ramp_torque_level(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_ramp_torque_level)
def get_exercise_mode(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_exercise_mode)
def get_disable_stair_ascent(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_disable_stair_ascent)
def get_amputation_type(wireless):
    return wireless.get_variable(wireless.bionics.var_power_knee_amputation_type)
def get_calibration_status(wireless):
    return wireless.get_variable(wireless.bionics.var_base_calibration_status)



########## RESET ##########
# Reset Functions
def reset_emulation(wireless):
    """ Stops emulation for all variables being emulated """
    wireless.stop_emulating()
def reset_pkm_biometric_stats_slr(wireless):
    """ Reset relative biometric stats database """
    wireless.action(wireless.bionics.action_pkm_biometric_stats_slr_reset)
def reset_user_parameters(wireless):
    """ Reset user parameters to their default value """
    wireless.action(wireless.bionics.action_pkm_user_parameters_reset)



########## DATABASES ##########
# Functionality to read the databases until we have reboot functionality in the virtual platform
def generate_database_functions():
    databaseFunctionsFile = 'tests/verification/DatabaseFunctions.py'
    b3cBiometricStatsDefFile = 'b3c_sw_platform/b3c_sw_def/B3C_SW_DEF_KneeBiometricStats.h'
    pkmBiometricStatsDefFile = 'pkm_app_def/PKM_APP_DB_BiometricParamsDef.h'

    variableSize = {'uint16_t' : ['2', 'H'], 'uint32_t' : ['4', 'I'], 'uint64_t' : ['8', 'Q'], 'float' : ['4', 'f']}

    # General function to create the contents for a database class
    def generate_db_content(db, allVariableTypes=0, fatalLog = False):
        content = '' # placeholder for the db content
        functions = '' # placeholder for the functions, they need to be at the end of the content
        for i, variable in enumerate(db):
            name = variable if not ' ' in variable else variable.split()[1] # variable name
            type = allVariableTypes if not ' ' in variable else variable.split()[0] # get the variable type for the size

            size, structType = variableSize[type] # get type and size of variable
            name = name.lower() if name.isupper() else name # set lower case because it's nicer to use
            # create line to store the start address of the variable and its size
            if i == 0: content += '\n\t\tself.' + name + ' = {"size" : ' + size + ' , "start" : self.startAdress}'
            else:
                previousname = db[i-1] if not ' ' in db[i-1] else db[i-1].split()[1]
                previousname = previousname.lower() if previousname.isupper() else previousname # set lower case because it's nicer to use
                content += ('\n\t\tself.' + name + ' = {"size" : ' + size + ', "start" : self.' + previousname + \
                            '["start"] + self.' + previousname + '["size"]}')
            # create line to store the end address of the variable
            content += ('\n\t\tself.' + name + '["end"] = self.' + name + '["start"] + self.' + name + '["size"]')
            # create line to get a get_variable function to use in the virtual platform
            functions += ('\n\tdef get_' + name + '(self, data): return struct.unpack(\'' + structType + '\', data[self.' + name + \
                        '["start"]:self.' + name + '["end"]])[0]')
        content += functions
        return content

    # Generate biometric stats functions
    def generate_biometric_stats(databaseStart):
        biometricStatsContent = ''

        b3cBiometricStats = re.findall(r'(?:uint32_t|uint64_t|float)\s\w*', open(b3cBiometricStatsDefFile, 'r', encoding='latin-1').read())
        pkmBiometricStats = re.findall(r'(?:uint32_t|uint64_t|float)\s\w*', open(pkmBiometricStatsDefFile, 'r', encoding='latin-1').read())
        absBiometricStats = b3cBiometricStats + pkmBiometricStats # absolute biometric stats
        relBiometricStats = [x + '_rel' for x in absBiometricStats] # relative biometric stats
        biometricStats = absBiometricStats + relBiometricStats

        biometricStatsContent += '\n\nclass BiometricStatsDb(): \n\tdef __init__(self):' # start the class
        biometricStatsContent += '\n\t\tself.startAdress = ' + str(int(databaseStart, 16)) # calculate the start address
        biometricStatsContent += generate_db_content(biometricStats) # the variable type is in the name of the variable
        return biometricStatsContent

    with open(databaseFunctionsFile, 'w+') as file:
        content = 'import struct' # start contents of the file
        content += generate_biometric_stats('0x0')
        file.write(content)



########## OTHER ##########
# Other Functions
def stop_emulating(wireless, index):
    """ Stops emulating a specific variable """
    wireless.stop_emulating(index)
def start_auto_adjustment(wireless):
    wireless.action(wireless.bionics.action_auto_adjustment)
def stop_logging(wireless):
    wireless.set_variable(wireless.bionics.var_base_logging_enabled, 0)

# Get input vector and general info of sensors
def get_system_inputs(resolution):
    """
        This functions parses through the input config file and returns a dataframe including the
        input names, their min and max fatal values and their indexes.
    """

    df = pd.DataFrame()

    minFatal = []
    maxFatal = []
    inputNames = []
    indexes = []
    lines = []

    cycles = 2
    length = np.pi * 2 * cycles

    for index in range(len(SInputConfigArr)):
        line = [x.strip() for x in SInputConfigArr[index].split(',')]
        inputName = line[-1][2:]

        # offset GCS
        if inputName == 'SYS_INPUT_GCS_0' or inputName == 'SYS_INPUT_GCS_2':
            minVal = round(float(line[6]),1) + 100
            maxVal = round(float(line[7]),1) - 100
        # offset gravity vector
        elif inputName == 'SYS_INPUT_SHANK_GRAV_X':
            minVal = round(float(line[6]),1) + 20
            maxVal = round(float(line[7]),1) - 45
        elif inputName == 'SYS_INPUT_SHANK_LIN_ACC_X':
            minVal = round(float(line[6]),1) + 10
            maxVal = round(float(line[7]),1) - 10
        else:
            minVal = round(float(line[6]),1)
            maxVal = round(float(line[7]),1)

        if minVal != 0 or maxVal != 0:
            minFatal.append(minVal)
            maxFatal.append(maxVal)
            inputNames.append(inputName)
            indexes.append(index)
            # make input values from 10% above min fatal and 10% below max fatal
            minValPrefix = 0.9 if minVal < 0 else 1.1
            maxValPrefix = 0.9
            # input a sine wave if inputs are knee angle or shank gravity, to ensure that we get non-stagnating values
            if inputName == 'SYS_INPUT_KNEE_ANGLE' or "SHANK_GRAV" in inputName or "LIN_ACC_X" in inputName:
                lines.append((minValPrefix*minVal + maxValPrefix*maxVal)/2 +  ((maxValPrefix*maxVal - minValPrefix*minVal)/2)*np.sin(np.arange(0, length, length / resolution)))
            else:
                lines.append(np.linspace(minValPrefix*minVal, maxValPrefix*maxVal, resolution))

    df['inputName'] = inputNames
    df['minFatal'] = minFatal
    df['maxFatal'] = maxFatal
    df['idx'] = indexes
    df['lines'] = lines

    return df

# Set input of sensors and tick the device
def get_and_set_device_input(device, resolution):

    df = get_system_inputs(resolution)

    # Set input of sensors
    for i in range(resolution):
        for j in range(df.shape[0]):
            device.set_input(df.idx[j], df.lines[j][i])
        device.tick()


if __name__ == '__main__':
    import pathlib
    import os
    from wireless_protocol_library import TcpCommunication, WirelessProtocolLibrary
    ROOT = pathlib.Path(os.path.dirname(os.path.abspath(__file__)))
    bionics_json_path = ROOT / "bionics.json"
    var_name_json_path = ROOT /"var_names.json"
    wireless = wireless = WirelessProtocolLibrary(TcpCommunication(), bionics_json_path) # Time out meaning that the power knee is not connected
    # test all functions under # Set Reactive AI Emulation 
    print(set_phase(wireless, 1))
    print(set_subphase(wireless, 2))
    print(set_activity(wireless, 3))
    print(set_torque(wireless, 4))
    print(set_knee_velocity(wireless, 5))
    print(set_knee_motor_velocity(wireless, 6))
    print(set_thigh_angle(wireless, 7))
    print(set_shank_angle(wireless, 8))
    print(set_knee_angle(wireless, 9))
    print(set_knee_motor_angle(wireless, 10))
    print(set_sagital_plane_offset_angle(wireless, 11))
    print(set_max_y_pos_in_window(wireless, 12))
    print(set_control_mode(wireless, 13))
    print(set_load_cell(wireless, 14))
    print(set_cadence(wireless, 15))
    print(set_toe_off_unload_point_found(wireless, 16))
    print(set_ct_knee_angle(wireless, 17))
    print(set_shank_rot_velocity(wireless, 18))
    print(set_thigh_rot_velocity(wireless, 19))
    print(set_brake_reached(wireless, 20))
    print(set_drift_status(wireless, 21))
    print(set_step_section(wireless, 22))
    print(set_toe_load(wireless, 23))
    print(set_dif_atan_grav(wireless, 24))

    