
import os, sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..','src')))
from connection.device import *

def verify_change_stance_flexion_level(wireless):
    set_activity            (wireless, value=Activity.ACTIVITY_FUMBLING_AROUND)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_REJECTION)
    set_cadence             (wireless, value=0)
    set_phase               (wireless, value=Phase.PHASE_STANCE)
    set_phase               (wireless, value=Phase.PHASE_SWING)
    set_subphase            (wireless, value=Subphase.SUBPHASE_BUMPER_AVOIDANCE)
    wireless.wait_ms(300)
    target_angle = get_target_angle(wireless)
    reset_emulation(wireless)
    return target_angle
    
def verify_change_toa_torque_level(wireless, torque1, torque2, torque3):
    set_load_cell           (wireless, value=0)
    set_input_knee_angle    (wireless, value=3)
    set_thigh_angle         (wireless, value=-16)
    set_torque              (wireless, value=0)
    set_load_cell           (wireless, value=51)
    set_toe_load            (wireless, value=41)
    wireless.wait_ms(500)
    set_torque              (wireless, value=torque1)
    set_thigh_angle         (wireless, value=0)
    set_shank_rot_velocity  (wireless, value=31)
    set_torque              (wireless, value=torque2)
    assert get_subphase(wireless) == Subphase.SUBPHASE_FORCE_REJECTION
    set_torque              (wireless, value=torque3)
    subphase = get_subphase(wireless)
    reset_emulation(wireless)
    return subphase
    
def verify_change_max_swing_flexion_angle_walking(wireless, knee_angle):
    set_cadence             (wireless, value=200)
    set_activity            (wireless, value=Activity.ACTIVITY_FORWARD_PROG)
    set_drift_status        (wireless, value=0)
    set_load_cell           (wireless, value=51)
    wireless.wait_ms(500)
    set_input_knee_angle    (wireless, value=3)
    wireless.wait_ms(100)
    set_load_cell           (wireless, value=0)
    set_thigh_angle         (wireless, value=-24)
    set_knee_velocity       (wireless, value=300)
    wireless.wait_ms(300)
    assert get_subphase(wireless) == Subphase.SUBPHASE_FORCE_FOLLOWING
    set_input_knee_angle    (wireless, value=knee_angle)
    subhpase = get_subphase(wireless)
    reset_emulation(wireless)
    return subhpase

def verify_change_sdn_and_ramp_torque_level(wireless, torque1, torque2):
    set_activity            (wireless, value=Activity.ACTIVITY_FUMBLING_AROUND)
    set_load_cell           (wireless, value=51)
    set_torque              (wireless, value=torque1)
    wireless.stop_emulating(Emulation.EMULATE_REACTIVE_AI_CURRENT_ACTIVITY)
    wireless.wait_ms(200)
    assert get_activity(wireless) == Activity.ACTIVITY_FUMBLING_AROUND
    set_torque              (wireless, value=torque2)
    activity = get_activity(wireless)
    reset_emulation(wireless)
    return activity

def verify_change_sdn_resistance_level(wireless):
    set_activity            (wireless, value=Activity.ACTIVITY_SIT_DOWN)
    wireless.wait_ms(500)
    set_load_cell           (wireless, value=51)
    wireless.wait_ms(200)
    cont_gain_km_one = get_gain_km(wireless)
    reset_emulation(wireless)
    set_activity            (wireless, value=Activity.ACTIVITY_DISSIPATION)
    set_load_cell           (wireless, value=51)
    wireless.wait_ms(500)
    cont_gain_km_two = get_gain_km(wireless)
    reset_emulation(wireless)
    return cont_gain_km_one, cont_gain_km_two

def verify_change_dis_resistance_level(wireless):
    set_activity            (wireless, value=Activity.ACTIVITY_SIT_DOWN)
    set_load_cell           (wireless, value=51)
    wireless.wait_ms(200)
    cont_gain_km = get_gain_km(wireless)
    reset_emulation(wireless)
    return cont_gain_km

def verify_change_sup_assistance_level(wireless):
    set_activity            (wireless, value=Activity.ACTIVITY_STAND_UP)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_REJECTION)
    set_load_cell           (wireless, value=51)
    wireless.wait_ms(350)
    set_load_cell           (wireless, value=0)
    set_load_cell           (wireless, value=51)
    cont_gain_kp_one = get_gain_kp(wireless)
    set_load_cell           (wireless, value=0)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_FOLLOWING)
    wireless.wait_ms(200)
    cont_gain_kp_two = get_gain_kp(wireless)
    reset_emulation(wireless)
    return cont_gain_kp_one, cont_gain_kp_two

def verify_change_dis_ext_speed(wireless):
    set_activity            (wireless, value=Activity.ACTIVITY_DISSIPATION)
    set_cadence             (wireless, value=0)
    set_load_cell           (wireless, value=51)
    wireless.wait_ms(500)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_REJECTION)
    set_load_cell           (wireless, value=0)
    wireless.wait_ms(500)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_FOLLOWING)
    cont_gain_kp_one = get_gain_kp(wireless)
    set_cadence             (wireless, value=200)
    set_load_cell           (wireless, value=51)
    wireless.wait_ms(500)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_REJECTION)
    set_load_cell           (wireless, value=0)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_FOLLOWING)
    cont_gain_kp_two = get_gain_kp(wireless)
    reset_emulation(wireless)
    return cont_gain_kp_one, cont_gain_kp_two

def verify_change_gen_assistance_level(wireless):
    set_activity            (wireless, value=Activity.ACTIVITY_GENERATION)
    set_cadence             (wireless, value=0)
    set_load_cell           (wireless, value=51)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_REJECTION)
    wireless.wait_ms(200)
    cont_gain_kp_one = get_gain_kp(wireless) 
    set_load_cell           (wireless, value=0)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_FOLLOWING)
    set_cadence             (wireless, value=200)
    set_load_cell           (wireless, value=51)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_REJECTION)
    wireless.wait_ms(200)
    cont_gain_kp_two = get_gain_kp(wireless) 
    reset_emulation(wireless)
    return cont_gain_kp_one, cont_gain_kp_two

def verify_change_gen_flexion_angle(wireless):
    set_activity            (wireless, value=Activity.ACTIVITY_GENERATION)
    set_cadence             (wireless, value=0)
    set_load_cell           (wireless, value=51)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_REJECTION)
    set_load_cell           (wireless, value=0)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_FOLLOWING)
    target_angle_one = get_target_angle(wireless)
    set_cadence             (wireless, value=200)
    set_load_cell           (wireless, value=51)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_REJECTION)
    set_load_cell           (wireless, value=0)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_FOLLOWING)
    target_angle_two = get_target_angle(wireless)
    reset_emulation(wireless)
    return target_angle_one, target_angle_two

def verify_change_gen_foot_placement_angle(wireless):
    set_activity            (wireless, value=Activity.ACTIVITY_GENERATION)
    set_cadence             (wireless, value=0)
    set_load_cell           (wireless, value=51)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_REJECTION)
    set_load_cell           (wireless, value=0)
    set_subphase            (wireless, value=Subphase.SUBPHASE_BUMPER_AVOIDANCE)
    target_angle_one = get_target_angle(wireless)
    set_cadence             (wireless, value=200)
    set_load_cell           (wireless, value=51)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_REJECTION)
    set_load_cell           (wireless, value=0)
    set_subphase            (wireless, value=Subphase.SUBPHASE_BUMPER_AVOIDANCE)
    target_angle_two = get_target_angle(wireless)
    reset_emulation(wireless)
    return target_angle_one, target_angle_two

def verify_change_disable_stair_ascent(wireless):
    set_activity            (wireless, value=Activity.ACTIVITY_FORWARD_PROG)
    set_phase               (wireless, value=Phase.PHASE_STANCE)
    set_phase               (wireless, value=Phase.PHASE_SWING)
    set_thigh_angle         (wireless, value=-31)
    set_input_lin_acc_y     (wireless, value=1)
    set_input_grav_x        (wireless, value=0.01)
    set_input_grav_y        (wireless, value=0.01)
    set_input_knee_angle    (wireless, value=100)
    set_phase               (wireless, value=Phase.PHASE_STANCE)
    wireless.wait_ms(300)
    wireless.stop_emulating(Emulation.EMULATE_REACTIVE_AI_CURRENT_ACTIVITY)
    assert get_activity(wireless) == Activity.ACTIVITY_FORWARD_PROG
    set_torque              (wireless, value=-4)
    wireless.wait_ms(100)
    activity = get_activity(wireless)
    reset_emulation(wireless)
    return activity

def auto_adjustment_steps(wireless):
    set_torque              (wireless, value=0)
    set_phase               (wireless, value=Phase.PHASE_STANCE)
    wireless.wait_ms(50)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_REJECTION)
    wireless.wait_ms(50)
    set_torque              (wireless, value=23)
    set_torque              (wireless, value=0)
    set_subphase            (wireless, value=Subphase.SUBPHASE_TOE_OFF_ASSIST)
    wireless.wait_ms(300)
    set_thigh_angle         (wireless, value=15)
    wireless.wait_ms(20)
    set_phase               (wireless, value=Phase.PHASE_SWING)
    wireless.wait_ms(320)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_FOLLOWING)
    wireless.wait_ms(20)
    set_subphase            (wireless, value=Subphase.SUBPHASE_BRAKE)
    wireless.wait_ms(100)
    set_subphase            (wireless, value=Subphase.SUBPHASE_FORCE_FOLLOWING)
    wireless.wait_ms(20)
    set_subphase            (wireless, value=Subphase.SUBPHASE_BUMPER_AVOIDANCE)
    wireless.wait_ms(100)


def var_power_knee_stance_flexion_level(wireless):
    reset_emulation(wireless)
    assert get_stance_flexion_level(wireless) == 50

    # set vars
    set_stance_flexion_level(wireless, value=0)
    assert get_stance_flexion_level(wireless) == 0

    # verify change
    target_angle = verify_change_stance_flexion_level(wireless)
    assert target_angle == 4+0/100*8 # target angle = 4 + var_power_knee_stance_flexion_level / 100 * 8
    
    # set vars
    set_stance_flexion_level(wireless, value=100)
    assert get_stance_flexion_level(wireless) == 100

    # verify change
    target_angle = verify_change_stance_flexion_level(wireless)
    assert target_angle == 4+100/100*8 # target angle = 4 + var_power_knee_stance_flexion_level / 100 * 8
    
def var_power_knee_toa_torque_level(wireless):
    reset_emulation(wireless)
    assert get_toa_torque_level(wireless) == 100

    # set vars
    set_toa_torque_level(wireless, value=0)
    assert get_toa_torque_level(wireless) == 0

    # verify change
    set_activity(wireless, Activity.ACTIVITY_FORWARD_PROG)
    assert verify_change_toa_torque_level(wireless, torque1=-16, torque2=-8, torque3=-7) == Subphase.SUBPHASE_TOE_OFF_ASSIST

    # set vars
    set_toa_torque_level(wireless, value=100)
    assert get_toa_torque_level(wireless) == 100

    # verify change
    set_activity(wireless, Activity.ACTIVITY_FUMBLING_AROUND)
    assert verify_change_toa_torque_level(wireless, torque1=-6, torque2=-1, torque3=11) == Subphase.SUBPHASE_TOE_OFF_ASSIST

def var_power_knee_max_swing_flexion_angle_walking(wireless):
    reset_emulation(wireless)
    assert get_swing_flexion_angle(wireless) == 60
    
    # set vars
    set_swing_flexion_angle(wireless, value=40)
    assert get_swing_flexion_angle(wireless) == 40

    # verify change
    assert verify_change_max_swing_flexion_angle_walking(wireless, knee_angle=41) == Subphase.SUBPHASE_BRAKE

    # set vars
    set_stance_flexion_level(wireless, value=100)
    assert get_stance_flexion_level(wireless) == 100
    set_swing_flexion_angle(wireless, value=76)
    assert get_swing_flexion_angle(wireless) == 75

    # verify change
    assert verify_change_max_swing_flexion_angle_walking(wireless, knee_angle=76) == Subphase.SUBPHASE_BRAKE
    
def var_power_knee_fa_fp_walking_speed(wireless):
    reset_emulation(wireless)
    assert wireless.get_variable(wireless.bionics.var_power_knee_fa_fp_walking_speed) == 50
    wireless.set_variable(wireless.bionics.var_power_knee_fa_fp_walking_speed, 1)
    assert wireless.get_variable(wireless.bionics.var_power_knee_fa_fp_walking_speed) == 1
    wireless.set_variable(wireless.bionics.var_power_knee_fa_fp_walking_speed, 100)
    assert wireless.get_variable(wireless.bionics.var_power_knee_fa_fp_walking_speed) == 100 

def var_power_knee_fa_fp_step_limit(wireless):
    reset_emulation(wireless)
    assert get_step_limit(wireless) == 2
    set_step_limit(wireless, value=0)
    assert get_step_limit(wireless) == 0
    set_step_limit(wireless, value=4)
    assert get_step_limit(wireless) == 4

def var_power_knee_sdn_torque_level(wireless):
    reset_emulation(wireless)
    assert get_sdn_torque_level(wireless) == 30
    assert get_ramp_torque_level(wireless) == 30 # assert ramp torque default here because we change it in this function before the other is called

    # set vars
    set_sdn_torque_level(wireless, value=5)
    assert get_sdn_torque_level(wireless) == 5

    # verify change
    verify_change_sdn_and_ramp_torque_level(wireless, torque1=4, torque2=6)

    # set vars
    set_sdn_torque_level(wireless, value=60)
    assert get_sdn_torque_level(wireless) == 60

    # verify change
    set_ramp_torque_level(wireless, value=60)
    assert verify_change_sdn_and_ramp_torque_level(wireless, torque1=59, torque2=61) == Activity.ACTIVITY_SIT_DOWN


def var_power_knee_ramp_torque_level(wireless):
    reset_emulation(wireless)

    # set vars
    set_ramp_torque_level(wireless, value=5)
    assert get_ramp_torque_level(wireless) == 5
    
    # verify change
    verify_change_sdn_and_ramp_torque_level(wireless, torque1=4, torque2=6)

    # set vars
    set_ramp_torque_level(wireless, value=60)
    assert get_ramp_torque_level(wireless) == 60

    # verify change
    set_sdn_torque_level(wireless, value=60)
    assert verify_change_sdn_and_ramp_torque_level(wireless, torque1=59, torque2=61) == Activity.ACTIVITY_SIT_DOWN

def var_power_knee_sdn_resistance_level(wireless):
    reset_emulation(wireless)
    assert get_sdn_resistance_level(wireless) == 50

    # set vars
    set_sdn_resistance_level(wireless, value=0)
    assert get_sdn_resistance_level(wireless) == 0
    
    # verify change
    cont_gain_km_one, cont_gain_km_two = verify_change_sdn_resistance_level(wireless)
    assert cont_gain_km_one == 0
    assert cont_gain_km_two == 0

    # set vars
    set_sdn_resistance_level(wireless, value=100)
    assert get_sdn_resistance_level(wireless) == 100

    # verify change
    cont_gain_km_one, cont_gain_km_two = verify_change_sdn_resistance_level(wireless)
    assert cont_gain_km_one == -100
    assert cont_gain_km_two == -100

def var_power_knee_dis_resistance_level(wireless):
    wl = wireless
    reset_emulation(wl)
    assert get_dis_resistance_level(wireless) == 50

    # set vars
    set_sdn_resistance_level(wireless, value=50)
    set_dis_resistance_level(wireless, value=0)
    assert get_dis_resistance_level(wireless) == 0
    
    # verify change
    cont_gain_km = verify_change_dis_resistance_level(wl)
    assert cont_gain_km == -50

    # set vars
    set_dis_resistance_level(wireless, value=100)
    assert get_dis_resistance_level(wireless) == 100
    
    # verify change
    cont_gain_km = verify_change_dis_resistance_level(wl)
    assert cont_gain_km == -50

def var_power_knee_sup_assistance_level(wireless):
    reset_emulation(wireless)
    assert get_sup_assistance_level(wireless) == 50

    # set vars
    set_sup_assistance_level(wireless, value=0)
    assert get_sup_assistance_level(wireless) == 0
    
    # verify change
    cont_gain_kp_one, cont_gain_kp_two = verify_change_sup_assistance_level(wireless)
    assert cont_gain_kp_one == 100
    assert cont_gain_kp_two == 100

    # set vars
    set_sup_assistance_level(wireless, value=100)
    assert get_sup_assistance_level(wireless) == 100
    
    # verify change
    cont_gain_kp_one, cont_gain_kp_two = verify_change_sup_assistance_level(wireless)
    assert cont_gain_kp_one == 300
    assert cont_gain_kp_two == 300

def var_power_knee_dis_ext_speed(wireless):
    reset_emulation(wireless)
    assert get_dis_extension_speed(wireless) == 50

    # set vars
    set_dis_extension_speed(wireless, value=0)
    assert get_dis_extension_speed(wireless) == 0

    # verify change
    cont_gain_kp_one, cont_gain_kp_two = verify_change_dis_ext_speed(wireless)
    assert cont_gain_kp_one == 0
    assert cont_gain_kp_two == 10

    # set vars
    set_dis_extension_speed(wireless, value=100)
    assert get_dis_extension_speed(wireless) == 100

    # verify change
    cont_gain_kp_one, cont_gain_kp_two = verify_change_dis_ext_speed(wireless)
    assert cont_gain_kp_one == 30
    assert cont_gain_kp_two == 40

def var_power_knee_gen_assistance_level(wireless):
    reset_emulation(wireless)
    assert get_gen_assistance_level(wireless) == 40

    # set vars
    set_gen_assistance_level(wireless, value=0)
    assert get_gen_assistance_level(wireless) == 0
    
    # verify change
    cont_gain_kp_one, cont_gain_kp_two = verify_change_gen_assistance_level(wireless)
    assert cont_gain_kp_one == 50
    assert cont_gain_kp_two == 50
    
    # set vars
    set_gen_assistance_level(wireless, value=100)
    assert get_gen_assistance_level(wireless) == 100
    
    # verify change
    cont_gain_kp_one, cont_gain_kp_two = verify_change_gen_assistance_level(wireless)
    assert cont_gain_kp_one == 350
    assert cont_gain_kp_two == 350

def var_power_knee_gen_flexion_angle(wireless):
    reset_emulation(wireless)
    assert get_gen_flexion_angle(wireless) == 70

    # set vars
    set_gen_flexion_angle(wireless, value=50)
    assert get_gen_flexion_angle(wireless) == 50
    
    # verify change
    target_angle_one, target_angle_two = verify_change_gen_flexion_angle(wireless)
    assert target_angle_one == 50
    assert target_angle_two == 50

    reset_emulation(wireless)

    # set vars
    set_gen_flexion_angle(wireless, value=100)
    assert get_gen_flexion_angle(wireless) == 100
    
    # verify change
    target_angle_one, target_angle_two = verify_change_gen_flexion_angle(wireless)
    assert target_angle_one == 100
    assert target_angle_two == 100

def var_power_knee_gen_foot_placement_angle(wireless):
    reset_emulation(wireless)
    assert get_gen_foot_placement_angle(wireless) == 55

    # set vars
    set_gen_foot_placement_angle(wireless, value=30)
    assert get_gen_foot_placement_angle(wireless) == 30
    
    # verify change
    target_angle_one, target_angle_two = verify_change_gen_foot_placement_angle(wireless)
    assert target_angle_one == 30
    assert target_angle_two == 30

    # set vars
    set_gen_foot_placement_angle(wireless, value=70)
    assert get_gen_foot_placement_angle(wireless) == 70
    
    # verify change
    target_angle_one, target_angle_two = verify_change_gen_foot_placement_angle(wireless)
    assert target_angle_one == 70
    assert target_angle_two == 70

def var_power_knee_disable_stair_ascent(wireless):
    reset_emulation(wireless)
    assert get_disable_stair_ascent(wireless) == 0

    # set vars
    set_disable_stair_ascent(wireless, value=0)
    assert get_disable_stair_ascent(wireless) == 0

    # verify change
    assert verify_change_disable_stair_ascent(wireless) == Activity.ACTIVITY_GENERATION

    # set vars
    set_disable_stair_ascent(wireless, value=1)
    assert get_disable_stair_ascent(wireless) == 1

    # verify change
    assert verify_change_disable_stair_ascent(wireless) == Activity.ACTIVITY_FORWARD_PROG

def var_leg_user_weight(wireless):
    reset_emulation(wireless)
    assert get_user_weight(wireless) == 80
    set_user_weight(wireless, value=30)
    assert get_user_weight(wireless) == 50
    set_user_weight(wireless, value=150)
    assert get_user_weight(wireless) == 116

def var_power_knee_amputation_type(wireless):
    reset_emulation(wireless)
    assert get_amputation_type(wireless) == AmputationType.AMPUTATION_TYPE_UNILATERAL
    set_amputation_type(wireless, value=AmputationType.AMPUTATION_TYPE_UNILATERAL)
    assert get_amputation_type(wireless) == AmputationType.AMPUTATION_TYPE_UNILATERAL

def var_base_feedback_vibration_intensity(wireless):
    reset_emulation(wireless)
    assert get_vibration_intensity(wireless) == 50
    
    set_vibration_intensity(wireless, value=1)
    assert get_vibration_intensity(wireless) == 1
    set_vibration_intensity(wireless, value=100)
    assert get_vibration_intensity(wireless) == 100

def var_power_knee_exercise_mode(wireless):
    reset_emulation(wireless)
    assert get_exercise_mode(wireless) == 0
    set_exercise_mode(wireless, value=1)
    assert get_exercise_mode(wireless) == 1
    assert wireless.get_variable(wireless.bionics.var_base_activity) == Activity.ACTIVITY_EXERCISE.value + 90 # Exercise is 10 in enum but 100 in code
    set_exercise_mode(wireless, value=0)
    assert get_exercise_mode(wireless) == 0
    assert wireless.get_variable(wireless.bionics.var_base_activity) != Activity.ACTIVITY_EXERCISE.value + 90 # Exercise is 10 in enum but 100 in code

def var_leg_user_thigh_length(wireless):
    reset_emulation(wireless)
    assert get_user_thigh_length(wireless) == 500
    set_user_thigh_length(wireless, value=99)
    assert get_user_thigh_length(wireless) == 100
    set_user_thigh_length(wireless, value=801)
    assert get_user_thigh_length(wireless) == 800
 
def var_leg_user_knee_height(wireless):
    reset_emulation(wireless)
    assert get_user_knee_height(wireless) == 500
    set_user_knee_height(wireless, value=99)
    assert get_user_knee_height(wireless) == 100
    set_user_knee_height(wireless, value=801)
    assert get_user_knee_height(wireless) == 800

def var_leg_user_foot_size(wireless):
    reset_emulation(wireless)
    assert get_user_foot_size(wireless) == 25
    set_user_foot_size(wireless, value=19)
    assert get_user_foot_size(wireless) == 20
    set_user_foot_size(wireless, value=31)
    assert get_user_foot_size(wireless) == 30

def var_power_knee_profile_id(wireless):
    reset_emulation(wireless)
    assert get_profile_id(wireless) == 1
    set_profile_id(wireless, value=0)
    assert get_profile_id(wireless) == 1
    set_profile_id(wireless, value=255)
    assert get_profile_id(wireless) == 255

def var_power_knee_ramp_resistance_level(wireless):
    reset_emulation(wireless)
    assert get_ramp_resistance_level(wireless) == 50
    set_ramp_resistance_level(wireless, value=0)
    assert get_ramp_resistance_level(wireless) == 0
    set_ramp_resistance_level(wireless, value=101)
    assert get_ramp_resistance_level(wireless) == 100

def var_base_calibration_status(wireless):
    reset_emulation(wireless)
    assert get_calibration_status(wireless) == 0

    set_ramp_torque_level(wireless, value=6)
    set_sdn_torque_level(wireless, value=6)
    set_sdn_resistance_level(wireless, value=6)
    set_dis_resistance_level(wireless, value=6)
    reset_emulation(wireless)

    start_auto_adjustment(wireless)

    set_activity(wireless, Activity.ACTIVITY_FORWARD_PROG)
    wireless.wait_ms(200)
    set_torque(wireless, value=0)

    # STEPS
    for _ in range(21):
        auto_adjustment_steps(wireless)

    assert get_calibration_status(wireless) == 1


def test_tsc_mainapp_d_6_user_parameters(wireless_library):
    wl = wireless_library
    reset_user_parameters(wl)
    
    # validate user parameters
    var_leg_user_weight(wl)
    var_power_knee_amputation_type(wl)
    var_base_feedback_vibration_intensity(wl)
    var_power_knee_stance_flexion_level(wl)
    var_power_knee_toa_torque_level(wl)
    var_power_knee_max_swing_flexion_angle_walking(wl)
    var_power_knee_fa_fp_walking_speed(wl)
    var_power_knee_fa_fp_step_limit(wl)
    var_power_knee_sdn_torque_level(wl)
    var_power_knee_ramp_torque_level(wl)
    var_power_knee_sdn_resistance_level(wl)
    var_power_knee_dis_resistance_level(wl)
    var_power_knee_sup_assistance_level(wl)
    var_power_knee_dis_ext_speed(wl)
    var_power_knee_gen_assistance_level(wl)
    var_power_knee_gen_flexion_angle(wl)
    var_power_knee_gen_foot_placement_angle(wl)
    var_power_knee_disable_stair_ascent(wl)
    var_power_knee_exercise_mode(wl)
    var_leg_user_thigh_length(wl)
    var_leg_user_knee_height(wl)
    var_leg_user_foot_size(wl)
    var_power_knee_profile_id(wl)
    var_power_knee_ramp_resistance_level(wl)
    # var_base_calibration_status(wl) # un-comment once Sturla fixes auto adjustment