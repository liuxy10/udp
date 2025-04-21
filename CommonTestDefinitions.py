import re
import os
from enum import IntEnum

ROOT = str(os.path.dirname(os.path.abspath(__file__)))

# Enums
activities          = re.findall('ACTIVITY_(?!\\w*(?:MODE))\\w*',
                                 open(ROOT + r'/pkm_app_def/PKM_APP_TaskDeliberativeAIDef.h', 'r', encoding='latin-1').read())
Activity            = IntEnum("Activity",       activities, start=0) # ACTIVITY_EXERCISE is 8 here but 100 in code!


amputationTypes     = re.findall('\\bAMPUTATION_TYPE\\w*',
                                 open(ROOT + r'/pkm_app_def/PKM_APP_TaskDeliberativeAIDef.h', 'r', encoding='latin-1').read())
AmputationType      = IntEnum("AmputationType", amputationTypes, start=1)


controlModes        = re.findall('\\bCONTROL_MODE\\w*',
                                 open(ROOT + r'/pkm_app_def/PKM_APP_TaskDeliberativeAIDef.h', 'r', encoding='latin-1').read())
ControlMode         = IntEnum("ControlMode",    controlModes, start=0)


emulationVariables  = re.findall('EMULATE\\w*',
                                 open(ROOT + r'/pkm_app_def/b3c_emulator_config.h', 'r', encoding='latin-1').read())
Emulation           = IntEnum("Emulation",      emulationVariables, start=0)


hwParameters        = re.findall('\\bHW_PARAM_\\w*',
                                 open(ROOT + r'/pkm_app_def/PKM_APP_DB_HardwareParamsDef.h', 'r', encoding='latin-1').read())
HwParameter         = IntEnum("HwParameter",  hwParameters, start=0)


phases              = re.findall('\\bPHASE\\w*',
                                 open(ROOT + r'/pkm_app_def/PKM_APP_ReactiveAIDef.h', 'r', encoding='latin-1').read())
Phase               = IntEnum("Phase",          phases, start=0)


profileIDs          = re.findall('\\bPROFILE_ID\\w*',
                                 open(ROOT + r'/pkm_app_def/PKM_APP_TaskDeliberativeAIDef.h', 'r', encoding='latin-1').read())
ProfileID           = IntEnum("ProfileID",      profileIDs, start=1)


stepSections        = re.findall('\\b(?:STANCE|SWING)_.\\w*',
                                 open(ROOT + r'/pkm_app_def/PKM_APP_TaskDeliberativeAIDef.h', 'r', encoding='latin-1').read())
StepSection         = IntEnum("StepSection",    stepSections, start=0)


subphases           = re.findall('\\bSUBPHASE\\w*',
                                 open(ROOT + r'/pkm_app_def/PKM_APP_ReactiveAIDef.h', 'r', encoding='latin-1').read())
Subphase            = IntEnum("Subphase",       subphases, start=0)


userParameters      = re.findall('\\bUSER_PARAM_(?!\\w*(?:DEFAULT))\\w*',
                                 open(ROOT + r'/pkm_app_def/PKM_APP_DB_UserParamsDef.h', 'r', encoding='latin-1').read())
UserParameter       = IntEnum("UserParameter",  userParameters, start=0)


# Sensor Input Config Array
SInputConfigArr     = re.findall(r'{SYS.*\bSYS_INPUT(?!_CONFIG)\w+', re.findall(r'SInput((.|\n)*)};',
                                 open(ROOT + r'/pkm_app_def/b3c_input_config.cpp', 'r', encoding='latin-1').read())[0][0])


# Plotting
coloursForPlotting = ['#FE6100', '#648FFF', '#DC267F', '#FFB000', '#DBD56E', '#A3C9A8', '#785EF0', '#334139']