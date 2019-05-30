import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as diagram

#   Definition of main function which reads input, compute this values and returns steering signal value
def FuzzControlSignal(error, errorChange, errorParameter, errorChangeParameter, uParameter):
    uControl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20, rule21, rule22, rule23, rule24, rule25, rule26, rule27, rule28, rule29, rule30, rule31, rule32, rule33, rule34, rule35, rule36, rule37, rule38, rule39, rule40, rule41, rule42, rule43, rule44, rule45, rule46, rule47, rule48, rule49])
    uControlSimulation = ctrl.ControlSystemSimulation(uControl)

    uControlSimulation.input['error'] = errorParameter*error
    uControlSimulation.input['errorChange'] = errorChangeParameter*errorChange

    uControlSimulation.compute()

    outputControlValue = uParameter*uControlSimulation.output['u']

    return outputControlValue

#   Definition of signals
error = ctrl.Antecedent(np.arange(-1, 1, 0.01), 'error')
errorChange = ctrl.Antecedent(np.arange(-1, 1, 0.01), 'errorChange')
u = ctrl.Consequent(np.arange(-1, 1, 0.01), 'u')

#   Definition of triangles
error.automf(7)
errorChange.automf(7)

u['superNegative'] = fuzz.trimf(u.universe, [-1.0, -1.0, -0.7])
u['veryNegative'] = fuzz.trimf(u.universe, [-1.0, -0.7, -0.4])
u['negative'] = fuzz.trimf(u.universe, [-0.7, -0.4, -0.1])
u['aBitNegative'] = fuzz.trimf(u.universe, [-0.4, -0.1, 0.0])
u['aboutZero'] = fuzz.trimf(u.universe, [-0.1, 0.0, 0.1])
u['aBitPositive'] = fuzz.trimf(u.universe, [0.0, 0.1, 0.4])
u['positive'] = fuzz.trimf(u.universe, [0.1, 0.4, 0.7])
u['veryPositive'] = fuzz.trimf(u.universe, [0.4, 0.7, 1.0])
u['superPositive'] = fuzz.trimf(u.universe, [0.7, 1.0, 1.0])

#Definition of rules
rule1 = ctrl.Rule(error['dismal'] & errorChange['dismal'], u['superNegative'])
rule2 = ctrl.Rule(error['poor'] & errorChange['dismal'], u['superNegative'])
rule3 = ctrl.Rule(error['mediocre'] & errorChange['dismal'], u['superNegative'])
rule4 = ctrl.Rule(error['average'] & errorChange['dismal'], u['veryNegative'])
rule5 = ctrl.Rule(error['decent'] & errorChange['dismal'], u['negative'])
rule6 = ctrl.Rule(error['good'] & errorChange['dismal'], u['aBitNegative'])
rule7 = ctrl.Rule(error['excellent'] & errorChange['dismal'], u['aboutZero'])

rule8 = ctrl.Rule(error['dismal'] & errorChange['poor'], u['superNegative'])
rule9 = ctrl.Rule(error['poor'] & errorChange['poor'], u['superNegative'])
rule10 = ctrl.Rule(error['mediocre'] & errorChange['poor'], u['veryNegative'])
rule11 = ctrl.Rule(error['average'] & errorChange['poor'], u['negative'])
rule12 = ctrl.Rule(error['decent'] & errorChange['poor'], u['aBitNegative'])
rule13 = ctrl.Rule(error['good'] & errorChange['poor'], u['aboutZero'])
rule14 = ctrl.Rule(error['excellent'] & errorChange['poor'], u['aBitPositive'])

rule15 = ctrl.Rule(error['dismal'] & errorChange['mediocre'], u['superNegative'])
rule16 = ctrl.Rule(error['poor'] & errorChange['mediocre'], u['veryNegative'])
rule17 = ctrl.Rule(error['mediocre'] & errorChange['mediocre'], u['negative'])
rule18 = ctrl.Rule(error['average'] & errorChange['mediocre'], u['aBitNegative'])
rule19 = ctrl.Rule(error['decent'] & errorChange['mediocre'], u['aboutZero'])
rule20 = ctrl.Rule(error['good'] & errorChange['mediocre'], u['aBitPositive'])
rule21 = ctrl.Rule(error['excellent'] & errorChange['mediocre'], u['positive'])

rule22 = ctrl.Rule(error['dismal'] & errorChange['average'], u['veryNegative'])
rule23 = ctrl.Rule(error['poor'] & errorChange['average'], u['negative'])
rule24 = ctrl.Rule(error['mediocre'] & errorChange['average'], u['aBitNegative'])
rule25 = ctrl.Rule(error['average'] & errorChange['average'], u['aboutZero'])
rule26 = ctrl.Rule(error['decent'] & errorChange['average'], u['aBitPositive'])
rule27 = ctrl.Rule(error['good'] & errorChange['average'], u['positive'])
rule28 = ctrl.Rule(error['excellent'] & errorChange['average'], u['veryPositive'])

rule29 = ctrl.Rule(error['dismal'] & errorChange['decent'], u['negative'])
rule30 = ctrl.Rule(error['poor'] & errorChange['decent'], u['aBitNegative'])
rule31 = ctrl.Rule(error['mediocre'] & errorChange['decent'], u['aboutZero'])
rule32 = ctrl.Rule(error['average'] & errorChange['decent'], u['aBitPositive'])
rule33 = ctrl.Rule(error['decent'] & errorChange['decent'], u['positive'])
rule34 = ctrl.Rule(error['good'] & errorChange['decent'], u['veryPositive'])
rule35 = ctrl.Rule(error['excellent'] & errorChange['decent'], u['superPositive'])

rule36 = ctrl.Rule(error['dismal'] & errorChange['good'], u['aBitNegative'])
rule37 = ctrl.Rule(error['poor'] & errorChange['good'], u['aboutZero'])
rule38 = ctrl.Rule(error['mediocre'] & errorChange['good'], u['aBitPositive'])
rule39 = ctrl.Rule(error['average'] & errorChange['good'], u['positive'])
rule40 = ctrl.Rule(error['decent'] & errorChange['good'], u['veryPositive'])
rule41 = ctrl.Rule(error['good'] & errorChange['good'], u['superPositive'])
rule42 = ctrl.Rule(error['excellent'] & errorChange['good'], u['superPositive'])

rule43 = ctrl.Rule(error['dismal'] & errorChange['excellent'], u['aboutZero'])
rule44 = ctrl.Rule(error['poor'] & errorChange['excellent'], u['aBitPositive'])
rule45 = ctrl.Rule(error['mediocre'] & errorChange['excellent'], u['positive'])
rule46 = ctrl.Rule(error['average'] & errorChange['excellent'], u['veryPositive'])
rule47 = ctrl.Rule(error['decent'] & errorChange['excellent'], u['superPositive'])
rule48 = ctrl.Rule(error['good'] & errorChange['excellent'], u['superPositive'])
rule49 = ctrl.Rule(error['excellent'] & errorChange['excellent'], u['superPositive'])