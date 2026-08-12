#Author - mawildoer
#Description - cycloidal drive generator

import adsk.core, adsk.fusion, adsk.cam, traceback
import math, sys

#Globals
app = adsk.core.Application.cast(None)
ui = adsk.core.UserInterface.cast(None)
units = ''

sys.stdout = open('D:\\temp\\python_output.txt', 'w')


#Command Inputs
reductionRatio = adsk.core.ValueCommandInput.cast(None)
depth = adsk.core.ValueCommandInput.cast(None)
rotorDia = adsk.core.ValueCommandInput.cast(None)
rollerDia = adsk.core.ValueCommandInput.cast(None)
eccentricity = adsk.core.TextBoxCommandInput.cast(None)
minWallThickness = adsk.core.ValueCommandInput.cast(None)
shaftDia = adsk.core.ValueCommandInput.cast(None)
bearingGuidance = adsk.core.TextBoxCommandInput.cast(None)
bearingID = adsk.core.ValueCommandInput.cast(None)
bearingOD = adsk.core.ValueCommandInput.cast(None)
bearingWidth = adsk.core.ValueCommandInput.cast(None)
numOutputHoles = adsk.core.IntegerSpinnerCommandInput.cast(None)
outputShaftDia = adsk.core.ValueCommandInput.cast(None)
outputBearingGuidance = adsk.core.TextBoxCommandInput.cast(None)
outputBearingDia = adsk.core.ValueCommandInput.cast(None)
minEdgeWallThickness = adsk.core.ValueCommandInput.cast(None)
outputPinCircleGuidance = adsk.core.TextBoxCommandInput.cast(None)
outputPinCircleDia = adsk.core.ValueCommandInput.cast(None)
validationStatus = adsk.core.TextBoxCommandInput.cast(None)

handlers = []

def getPoint(t, R, Rr, E, N):
    #psi = -math.atan(math.sin((1 - N) * theta) / ((R / (E * N)) - math.cos((1 - N) * theta)))
    #x = R * math.cos(theta) - Rr * math.cos(theta - psi) - E * math.cos(N * theta)
    #y =  - R * math.sin(theta) + Rr * math.sin(theta - psi) + E * math.cos(N * theta)
    psi = math.atan2(math.sin((1-N)*t), ((R/(E*N))-math.cos((1-N)*t)))

    x = (R*math.cos(t))-(Rr*math.cos(t+psi))-(E*math.cos(N*t))
    y = (-R*math.sin(t))+(Rr*math.sin(t+psi))+(E*math.sin(N*t))
    #x = (10*math.cos(t))-(1.5*math.cos(t+math.atan(math.sin(-9*t)/((4/3)-math.cos(-9*t)))))-(0.75*math.cos(10*t))
    #y = (-10*math.sin(t))+(1.5*math.sin(t+math.atan(math.sin(-9*t)/((4/3)-math.cos(-9*t)))))+(0.75*math.sin(10*t))
    return (x,y)


def getDist(xa, ya, xb, yb):
    return math.sqrt((xa-xb)**2 + (ya-yb)**2)


def addOrUpdateParam(params, name, value, unit, comment):
    """Create a user parameter, or update it in place if it already exists.

    UserParameters.add() raises RuntimeError('3 : param name is not valid')
    if a parameter with that name already exists in the design, so re-running
    this script on a document from a previous run would otherwise crash here.
    """
    existing = params.itemByName(name)
    if existing:
        existing.value = value
        existing.comment = comment
        return existing
    return params.add(name, adsk.core.ValueInput.createByReal(value), unit, comment)


def calcEcc(rotorDia, reductionRatio):
    """Calculate the Eccentricity

    Args:
        rotorDia (float): The diameter of the rotor
        reductionRatio (int): The reduction ratio of the gearbox

    Returns:
        eccentricity: float
    """
    rotorRadius = rotorDia/2
    NumberOfRollers = reductionRatio + 1
    housing_cir = 2 * rotorRadius * math.pi
    rollerRadius = housing_cir / (4 * NumberOfRollers)
    eccentricity = 0.5 * rollerRadius
    return eccentricity, rollerRadius


def calcMinBearingBore(shaftDia, rotorDia, reductionRatio, minWall):
    """Minimum recommended bearing bore (ID) for the input eccentric, in mm.

    Independently recomputes eccentricity from rotorDia/reductionRatio
    rather than reusing calcEcc's return value, so it stays correct even
    if calcEcc's convention ever changes again. shaftDia/rotorDia are true
    cm (straight off ValueCommandInput.value); minWall is the user-tunable
    minimum wall thickness (Minimum Wall Thickness input), in mm.
    """
    rotorRadius = rotorDia / 2
    numberOfRollers = reductionRatio + 1
    housingCir = 2 * rotorRadius * math.pi
    rollerRadius = housingCir / (4 * numberOfRollers)
    ecc = 0.5 * rollerRadius
    return (shaftDia * 10) + (2 * ecc * 10) + (2 * minWall)


def calcMinOutputBearingOD(outputShaftDia, minWall):
    """Minimum recommended output bearing outer diameter (OD), in mm.

    Unlike the input eccentric, the output bearing's bore sits concentric
    on its shaft (no offset -- see commandExecuteHandler), so it just
    needs to match the output shaft diameter, and the OD only needs to
    leave at least minWall of wall thickness around that bore. outputShaftDia
    is true cm (straight off ValueCommandInput.value); minWall is the
    user-tunable minimum wall thickness (Minimum Wall Thickness input), in mm.
    """
    return (outputShaftDia * 10) + (2 * minWall)


def calcMinLobeRadius(rotorRadius, rollerRadius, ecc, numberOfRollers, samples=200):
    """Minimum distance from the rotor's own local origin to its lobe boundary.

    The lobe profile has no simple closed-form inner radius, so this
    numerically samples the same getPoint() curve used to build the real
    lobe geometry over one lobe period. All args/return are true cm.
    """
    et = 2 * math.pi / (numberOfRollers - 1)
    minRadius = None
    for i in range(samples + 1):
        t = et * i / samples
        x, y = getPoint(t, rotorRadius, rollerRadius, ecc, numberOfRollers)
        r = math.sqrt(x * x + y * y)
        if minRadius is None or r < minRadius:
            minRadius = r
    return minRadius


def calcOutputPinCircleRange(rotorDia, reductionRatio, bearingOD, outputBearingDia, numOutputHoles, minEdgeWallThickness):
    """Valid range for the Output Pin Circle Diameter, in mm: (minDia, maxDia).

    The minimum uses the same exact formulas as the hard validation guards
    in validateGeometry (no output holes overlapping each other or the
    central input bearing hole -- since the output holes are concentric
    with the input hole, this distance is now exact, not a worst case).
    The maximum keeps at least minEdgeWallThickness between every output
    hole and the lobe boundary; it's still necessarily conservative in one
    sense, since it treats the lobe profile's closest approach to the
    rotor's own centre (see calcMinLobeRadius) as if it were a full circle,
    which is always safe but not always the tightest possible fit. All
    Dia/OD args are true cm (straight off ValueCommandInput.value);
    minEdgeWallThickness is in mm.
    """
    rotorRadius = rotorDia / 2
    numberOfRollers = reductionRatio + 1
    housingCir = 2 * rotorRadius * math.pi
    rollerRadius = housingCir / (4 * numberOfRollers)
    ecc = 0.5 * rollerRadius

    bearingODRadius = bearingOD / 2
    outputBearingRadius = outputBearingDia / 2
    outputHoleRadius = outputBearingRadius + ecc

    minRadiusFromCentre = bearingODRadius + outputHoleRadius
    minRadiusFromAdjacent = outputHoleRadius / math.sin(math.pi / numOutputHoles)
    minRadius = max(minRadiusFromCentre, minRadiusFromAdjacent)

    minLobeRadius = calcMinLobeRadius(rotorRadius, rollerRadius, ecc, numberOfRollers)
    maxRadius = minLobeRadius - outputHoleRadius - (minEdgeWallThickness / 10)

    return (minRadius * 20, maxRadius * 20)


def validateGeometry(rotorDia, reductionRatio, shaftDia, bearingID, bearingOD,
                      outputShaftDia, outputBearingDia, outputPinCircleDia, numOutputHoles,
                      minEdgeWallThickness):
    """Check whether the current dimensions produce buildable geometry.

    Returns (True, '') if everything fits, or (False, reason) naming the
    first problem found. Mirrors the geometry actually built in
    commandExecuteHandler exactly, so a True result here is guaranteed to
    build successfully there. Used both to live-gate the dialog's OK
    button (commandValidateInputsHandler) and as a defensive fallback
    check in commandExecuteHandler itself. All *Dia/*ID/*OD args are true
    cm (straight off ValueCommandInput.value); numOutputHoles is a plain
    int; minEdgeWallThickness is in mm.
    """
    rotorRadius = rotorDia / 2
    numberOfRollers = reductionRatio + 1
    housingCir = 2 * rotorRadius * math.pi
    rollerRadius = housingCir / (4 * numberOfRollers)
    ecc = 0.5 * rollerRadius

    shaftRadius = shaftDia / 2
    bearingIDRadius = bearingID / 2
    bearingODRadius = bearingOD / 2

    # The eccentric bushing's bore must nest fully inside its OD once offset
    # by ecc, leaving positive wall thickness at the thinnest point.
    if (bearingIDRadius - ecc - shaftRadius) <= 0:
        return False, 'Bearing ID is too small for the Input Shaft Diameter and the current eccentricity.'

    if (bearingODRadius - bearingIDRadius) <= 0:
        return False, 'Bearing OD must be larger than Bearing ID.'

    outputShaftRadius = outputShaftDia / 2
    outputBearingRadius = outputBearingDia / 2
    outputPinCircleRadius = outputPinCircleDia / 2
    outputHoleRadius = outputBearingRadius + ecc

    if (outputBearingRadius - outputShaftRadius) <= 0:
        return False, 'Output Bearing Diameter must be larger than Output Shaft Diameter.'

    # The output holes are concentric with the input hole (both centred on the
    # rotor's own rotational centre), so this clearance is exact for every hole.
    adjacentHoleChord = 2 * outputPinCircleRadius * math.sin(math.pi / numOutputHoles)
    if adjacentHoleChord <= 2 * outputHoleRadius:
        return False, 'Output holes would overlap each other. Increase Output Pin Circle Diameter, reduce Number of Output Holes, or reduce Output Bearing Diameter.'

    if outputPinCircleRadius - outputHoleRadius <= bearingODRadius:
        return False, 'Output holes would overlap the central input bearing hole. Increase Output Pin Circle Diameter, or reduce Output Bearing Diameter / Bearing OD.'

    # Every hole cut into the rotor (input and output) must keep at least
    # minEdgeWallThickness of material between it and the rotor's outer (lobe)
    # boundary. minLobeRadius is the closest the lobe boundary ever gets to
    # the rotor's own centre -- a conservative, angle-independent bound.
    minWallCm = minEdgeWallThickness / 10
    minLobeRadius = calcMinLobeRadius(rotorRadius, rollerRadius, ecc, numberOfRollers)

    if minLobeRadius - bearingODRadius < minWallCm:
        return False, 'The input bearing hole is too close to the rotor\'s outer edge. Reduce Bearing OD, increase Roller PCD, or reduce Minimum Wall Thickness (Holes to Outer Edge).'

    if minLobeRadius - outputPinCircleRadius - outputHoleRadius < minWallCm:
        return False, 'The output holes are too close to the rotor\'s outer edge. Reduce Output Pin Circle Diameter / Output Bearing Diameter, increase Roller PCD, or reduce Minimum Wall Thickness (Holes to Outer Edge).'

    return True, ''



def run(context):
    """Fusion starts here"""
    print("=== run ====")
    try:
        global app, ui
        app = adsk.core.Application.get()
        ui  = app.userInterface
        des = adsk.fusion.Design.cast(app.activeProduct)


        cmdDef = ui.commandDefinitions.itemById('adskCycloidPythonScript')
        if not cmdDef:
            # Create a command definition.
            cmdDef = ui.commandDefinitions.addButtonDefinition('adskCycloidPythonScript', 'Cycloid', 'Creates a cycloid gearbox') 
        
        # Connect to the command created event.
        onCommandCreated = commandCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        handlers.append(onCommandCreated)
        
        # Execute the command.
        cmdDef.execute()

        # prevent this module from being terminate when the script returns, because we are waiting for event handlers to fire
        adsk.autoTerminate(False)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))




# Event handler for the commandCreated event.
class commandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            print('--- commandCreatedHandler ---')
            eventArgs = adsk.core.CommandCreatedEventArgs.cast(args)
            
            # Verify that a Fusion design is active.
            des = adsk.fusion.Design.cast(app.activeProduct)
            if not des:
                ui.messageBox('A Fusion design must be active when invoking this command.')
                return()
                
            defaultUnits = des.unitsManager.defaultLengthUnits
                
            # Determine whether to use inches or millimeters as the intial default.
            global units
            if defaultUnits == 'in' or defaultUnits == 'ft':
                units = 'in'
            else:
                units = 'mm'
                        
            print(f'Units: {units}')
            
            cmd = eventArgs.command
            cmd.isExecutedWhenPreEmpted = False
            inputs = cmd.commandInputs
            
            global reductionRatio, depth, rotorDia, eccentricity, rollerDia, minWallThickness, shaftDia, bearingGuidance, bearingID, bearingOD, bearingWidth, numOutputHoles, outputShaftDia, outputBearingGuidance, outputBearingDia, minEdgeWallThickness, outputPinCircleGuidance, outputPinCircleDia, validationStatus

            # Define the command dialog.

            reductionRatio = inputs.addIntegerSpinnerCommandInput('reductionRatio', 'Reduction Ration (x:1)', 3, 100, 1, 30)
            rotorDia = inputs.addValueInput('rotorDia', 'Roller Pitch Circle Diameter (PCD)', 'mm', adsk.core.ValueInput.createByReal(10.0))
            depth = inputs.addValueInput('depth', 'Gear Height', 'mm', adsk.core.ValueInput.createByReal(1.0))
            rollerDia = inputs.addValueInput('rollerDia', 'Roller Diameter', 'mm', adsk.core.ValueInput.createByReal(0.0))
            eccentricity = inputs.addTextBoxCommandInput('eccentricity', 'Eccentricity', '', 1, True)
            minWallThickness = inputs.addValueInput('minWallThickness', 'Minimum Wall Thickness', 'mm', adsk.core.ValueInput.createByString('1.5 mm'))
            shaftDia = inputs.addValueInput('shaftDia', 'Input Shaft Diameter', 'mm', adsk.core.ValueInput.createByString('5 mm'))
            bearingGuidance = inputs.addTextBoxCommandInput('bearingGuidance', 'Min. Recommended Bearing Bore', '', 1, True)
            bearingID = inputs.addValueInput('bearingID', 'Bearing Bore (ID)', 'mm', adsk.core.ValueInput.createByString('8 mm'))
            bearingOD = inputs.addValueInput('bearingOD', 'Bearing Outer Diameter (OD)', 'mm', adsk.core.ValueInput.createByString('22 mm'))
            bearingWidth = inputs.addValueInput('bearingWidth', 'Bearing Width', 'mm', adsk.core.ValueInput.createByString('7 mm'))
            numOutputHoles = inputs.addIntegerSpinnerCommandInput('numOutputHoles', 'Number of Output Holes', 3, 20, 1, 3)
            outputShaftDia = inputs.addValueInput('outputShaftDia', 'Output Shaft Diameter', 'mm', adsk.core.ValueInput.createByString('5 mm'))
            outputBearingGuidance = inputs.addTextBoxCommandInput('outputBearingGuidance', 'Min. Recommended Output Bearing OD', '', 1, True)
            outputBearingDia = inputs.addValueInput('outputBearingDia', 'Output Bearing Diameter (OD)', 'mm', adsk.core.ValueInput.createByString('10 mm'))
            minEdgeWallThickness = inputs.addValueInput('minEdgeWallThickness', 'Minimum Wall Thickness (Holes to Outer Edge)', 'mm', adsk.core.ValueInput.createByString('1.5 mm'))
            outputPinCircleGuidance = inputs.addTextBoxCommandInput('outputPinCircleGuidance', 'Output Pin Circle Diameter Range', '', 2, True)
            outputPinCircleDia = inputs.addValueInput('outputPinCircleDia', 'Output Pin Circle Diameter', 'mm', adsk.core.ValueInput.createByString('50 mm'))
            validationStatus = inputs.addTextBoxCommandInput('validationStatus', 'Validation Status', '', 2, True)

            # re-calculate the eccentricity and update the UI.
            Ecc, rollerRadius = calcEcc(rotorDia.value, reductionRatio.value)
            eccentricity.text = str(round(Ecc, 3)*10) + 'mm'
            print(f'eccentricity: {eccentricity.text}')

            # calculate the roller radius
            rollerDia.value = rollerRadius * 2

            # re-calculate the minimum recommended bearing bore and update the UI.
            minBore = calcMinBearingBore(shaftDia.value, rotorDia.value, reductionRatio.value, minWallThickness.value * 10)
            bearingGuidance.text = str(round(minBore, 2)) + 'mm'
            print(f'bearingGuidance: {bearingGuidance.text}')

            # re-calculate the minimum recommended output bearing OD and update the UI.
            minOutputOD = calcMinOutputBearingOD(outputShaftDia.value, minWallThickness.value * 10)
            outputBearingGuidance.text = str(round(minOutputOD, 2)) + 'mm'
            print(f'outputBearingGuidance: {outputBearingGuidance.text}')

            # re-calculate the valid Output Pin Circle Diameter range and update the UI.
            minPinCircle, maxPinCircle = calcOutputPinCircleRange(
                rotorDia.value, reductionRatio.value, bearingOD.value, outputBearingDia.value,
                numOutputHoles.value, minEdgeWallThickness.value * 10)
            if minPinCircle <= maxPinCircle:
                outputPinCircleGuidance.text = '{}mm - {}mm'.format(round(minPinCircle, 1), round(maxPinCircle, 1))
            else:
                outputPinCircleGuidance.text = 'No valid diameter with current dimensions -- adjust other values'
            print(f'outputPinCircleGuidance: {outputPinCircleGuidance.text}')

            # Connect to the command related events.
            onExecute = commandExecuteHandler()
            cmd.execute.add(onExecute)
            handlers.append(onExecute)        
            
            onInputChanged = commandInputChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            handlers.append(onInputChanged)     
            
            onValidateInputs = commandValidateInputsHandler()
            cmd.validateInputs.add(onValidateInputs)
            handlers.append(onValidateInputs)

            onDestroy = commandDestroyHandler()
            cmd.destroy.add(onDestroy)
            handlers.append(onDestroy)
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))




# Event handler for the inputChanged event.
class commandInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            print("--- commandInputChangedHandler ---")
            eventArgs = adsk.core.InputChangedEventArgs.cast(args)
            changedInput = eventArgs.input
            
            global eccentricity, rollerDia, rotorDia, minWallThickness, shaftDia, bearingGuidance, bearingOD
            global outputShaftDia, outputBearingGuidance, outputBearingDia, minEdgeWallThickness
            global outputPinCircleGuidance, numOutputHoles

            def updateBearingGuidance():
                minBore = calcMinBearingBore(shaftDia.value, rotorDia.value, reductionRatio.value, minWallThickness.value * 10)
                bearingGuidance.text = str(round(minBore, 2)) + 'mm'

            def updateOutputBearingGuidance():
                minOutputOD = calcMinOutputBearingOD(outputShaftDia.value, minWallThickness.value * 10)
                outputBearingGuidance.text = str(round(minOutputOD, 2)) + 'mm'

            def updateOutputPinCircleGuidance():
                minPinCircle, maxPinCircle = calcOutputPinCircleRange(
                    rotorDia.value, reductionRatio.value, bearingOD.value, outputBearingDia.value,
                    numOutputHoles.value, minEdgeWallThickness.value * 10)
                if minPinCircle <= maxPinCircle:
                    outputPinCircleGuidance.text = '{}mm - {}mm'.format(round(minPinCircle, 1), round(maxPinCircle, 1))
                else:
                    outputPinCircleGuidance.text = 'No valid diameter with current dimensions -- adjust other values'

            if changedInput.id == 'rotorDia' or changedInput.id == 'reductionRatio':
                # re-calculate the eccentricity and update the UI.
                Ecc, rollerRadius = calcEcc(rotorDia.value, reductionRatio.value)
                eccentricity.text = str(round(Ecc, 3)*10) + 'mm'
                rollerDia.value = rollerRadius * 2
                print(f'Eccentricity has changed to: {eccentricity.text}')

                updateBearingGuidance()
                updateOutputPinCircleGuidance()

            if changedInput.id == 'rollerDia':
                # re-calculate the rotor Dia
                housingDia = (rollerDia.value / 2) * (4 * (reductionRatio.value + 1))
                rotorDia.value = (housingDia / math.pi)

            if changedInput.id == 'shaftDia':
                updateBearingGuidance()
                print(f'bearingGuidance has changed to: {bearingGuidance.text}')

            if changedInput.id == 'minWallThickness':
                updateBearingGuidance()
                updateOutputBearingGuidance()

            if changedInput.id == 'bearingOD':
                updateOutputPinCircleGuidance()

            if changedInput.id == 'outputShaftDia':
                updateOutputBearingGuidance()
                print(f'outputBearingGuidance has changed to: {outputBearingGuidance.text}')

            if changedInput.id in ('outputBearingDia', 'numOutputHoles', 'minEdgeWallThickness'):
                updateOutputPinCircleGuidance()
                print(f'outputPinCircleGuidance has changed to: {outputPinCircleGuidance.text}')


        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


# Event handler for the validateInputs event -- gates the OK button so an
# invalid combination of dimensions keeps the dialog open (with a status
# message) instead of letting the user click OK and have the command exit.
class commandValidateInputsHandler(adsk.core.ValidateInputsEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        eventArgs = adsk.core.ValidateInputsEventArgs.cast(args)
        try:
            global rotorDia, reductionRatio, shaftDia, bearingID, bearingOD
            global outputShaftDia, outputBearingDia, outputPinCircleDia, numOutputHoles, validationStatus, minEdgeWallThickness

            isValid, reason = validateGeometry(
                rotorDia.value, reductionRatio.value, shaftDia.value, bearingID.value, bearingOD.value,
                outputShaftDia.value, outputBearingDia.value, outputPinCircleDia.value, numOutputHoles.value,
                minEdgeWallThickness.value * 10)

            eventArgs.areInputsValid = isValid
            validationStatus.text = '' if isValid else 'Cannot build: ' + reason
        except:
            # Fail open -- a bug in validation shouldn't permanently lock the OK button.
            eventArgs.areInputsValid = True
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))





# Event handler for the execute event.
class commandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            print("--- commandExecuteHandler ---")
            eventArgs = adsk.core.CommandEventArgs.cast(args)

            
            # Save the current values as attributes.
            des = adsk.fusion.Design.cast(app.activeProduct)
            params = des.userParameters
            addOrUpdateParam(params, 'reductionRatio', reductionRatio.value, '', 'Reduction Radion (x:1)')
            addOrUpdateParam(params, 'gearHeight', depth.value, 'mm', 'Extrude height')
            addOrUpdateParam(params, 'rotorDia', rotorDia.value, 'mm', 'Pitch circle diameter (PCD) of the housing rollers')
            addOrUpdateParam(params, 'rollerDia', rollerDia.value, 'mm', 'Diameter of the roller pins')
            addOrUpdateParam(params, 'shaftDia', shaftDia.value, 'mm', 'Input shaft diameter')
            addOrUpdateParam(params, 'bearingID', bearingID.value, 'mm', 'Input bearing bore (ID)')
            addOrUpdateParam(params, 'bearingOD', bearingOD.value, 'mm', 'Input bearing outer diameter (OD)')
            addOrUpdateParam(params, 'bearingWidth', bearingWidth.value, 'mm', 'Input bearing width')
            addOrUpdateParam(params, 'minWallThickness', minWallThickness.value, 'mm', 'Minimum wall thickness (eccentric/output bearing walls)')
            addOrUpdateParam(params, 'numOutputHoles', numOutputHoles.value, '', 'Number of output holes/bearings')
            addOrUpdateParam(params, 'outputShaftDia', outputShaftDia.value, 'mm', 'Output shaft diameter')
            addOrUpdateParam(params, 'outputBearingDia', outputBearingDia.value, 'mm', 'Output bearing outer diameter (OD)')
            addOrUpdateParam(params, 'minEdgeWallThickness', minEdgeWallThickness.value, 'mm', 'Minimum wall thickness (holes to outer edge)')
            addOrUpdateParam(params, 'outputPinCircleDia', outputPinCircleDia.value, 'mm', 'Output pin circle diameter')



            # - The values below are all in CM as this is the default internal unit of Fusion 360
            rotorThickness = depth.value
            housingThickness = 1 * rotorThickness
            rotorRadius = (rotorDia.value)/2                          # rotor radius (cm)
            numberOfRollers = reductionRatio.value + 1                # number of rollers
            
            # calculated values
            housing_cir = 2 * rotorRadius * math.pi
            rollerRadius = housing_cir / (4 * numberOfRollers)
            print(f'roller radius: {rollerRadius}')
            ecc = 0.5 * rollerRadius                                # eccentricity
            maxDist = 0.25 * rollerRadius                           # maximum allowed distance between points
            minDist = 0.5 * maxDist                                 # the minimum allowed distance between points

            # --- Input eccentric / bearing geometry (all values cm) ---
            shaftRadius = shaftDia.value / 2
            bearingIDRadius = bearingID.value / 2
            bearingODRadius = bearingOD.value / 2
            bearingWidthCm = bearingWidth.value

            # --- Output holes / output bearings (all values cm) ---
            outputShaftRadius = outputShaftDia.value / 2            # also used as the output bearing's bore radius (nominal fit)
            outputBearingRadius = outputBearingDia.value / 2
            outputPinCircleRadius = outputPinCircleDia.value / 2
            # The output hole must be oversized relative to the output bearing's OD by
            # exactly 2x the eccentricity -- this is what lets the rotor's wobble be
            # absorbed while only the pure rotation is transmitted to the output pins.
            outputHoleRadius = outputBearingRadius + ecc

            # Validate before building anything. This mirrors the exact same checks
            # already used to gate the dialog's OK button (see
            # commandValidateInputsHandler) -- kept here too as a defensive fallback,
            # since normally the OK button should already be disabled if this would fail.
            isValid, reason = validateGeometry(
                rotorDia.value, reductionRatio.value, shaftDia.value, bearingID.value, bearingOD.value,
                outputShaftDia.value, outputBearingDia.value, outputPinCircleDia.value, numOutputHoles.value,
                minEdgeWallThickness.value * 10)
            if not isValid:
                ui.messageBox('Cannot build: ' + reason)
                return()

            root = des.rootComponent

            rotorOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            # Fusion auto-grounds (anchors) the first occurrence created in an
            # otherwise-empty design, to give it a fixed spatial reference. That
            # grounding fights the eccentricity transform set further down, pinning
            # the rotor to the origin instead of letting it sit at (ecc, 0). Un-ground
            # it so the transform actually takes effect.
            rotorOcc.isGrounded = False
            rotor = rotorOcc.component
            rotor.name = 'rotor'

            sk = rotor.sketches.add(root.xYConstructionPlane)

            points = adsk.core.ObjectCollection.create()

            #ui.messageBox('Eccentricity ' + str(E/10) + "mm")
            addOrUpdateParam(params, 'Eccentricity', ecc, 'mm', 'Eccentricity')

            (xs, ys) = getPoint(0, rotorRadius, rollerRadius, ecc, numberOfRollers)
            points.add(adsk.core.Point3D.create(xs,ys,0))

            et = 2 * math.pi / (numberOfRollers-1)
            (xe, ye) = getPoint(et, rotorRadius, rollerRadius, ecc, numberOfRollers)
            x = xs
            y = ys
            dist = 0
            ct = 0
            dt = math.pi / numberOfRollers
            numPoints = 0

            while ((math.sqrt((x-xe)**2 + (y-ye)**2) > maxDist or ct < et/2) and ct < et): #close enough to the end to call it, but over half way
            #while (ct < et/80): #close enough to the end to call it, but over half way
                (xt, yt) = getPoint(ct+dt, rotorRadius, rollerRadius, ecc, numberOfRollers)
                dist = getDist(x, y, xt, yt)

                ddt = dt/2
                lastTooBig = False
                lastTooSmall = False

                while (dist > maxDist or dist < minDist):
                    if (dist > maxDist):
                        if (lastTooSmall):
                            ddt /= 2

                        lastTooSmall = False
                        lastTooBig = True

                        if (ddt > dt/2):
                            ddt = dt/2

                        dt -= ddt

                    elif (dist < minDist):
                        if (lastTooBig):
                            ddt /= 2

                        lastTooSmall = True
                        lastTooBig = False
                        dt += ddt


                    (xt, yt) = getPoint(ct+dt, rotorRadius, rollerRadius, ecc, numberOfRollers)
                    dist = getDist(x, y, xt, yt)

                x = xt
                y = yt
                points.add(adsk.core.Point3D.create(x,y,0))
                numPoints += 1
                ct += dt

            points.add(adsk.core.Point3D.create(xe,ye,0))
            crv = sk.sketchCurves.sketchFittedSplines.add(points)

            lines = sk.sketchCurves.sketchLines;
            line1 = lines.addByTwoPoints(adsk.core.Point3D.create(0, 0, 0), crv.startSketchPoint)
            line2 = lines.addByTwoPoints(line1.startSketchPoint, crv.endSketchPoint)

            prof = sk.profiles.item(0)
            distance = adsk.core.ValueInput.createByReal(rotorThickness)

            # Get extrude features
            extrudes = rotor.features.extrudeFeatures
            extrude1 = extrudes.addSimple(prof, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

            # Get the extrusion body
            body1 = extrude1.bodies.item(0)
            body1.name = "rotor"

            inputEntites = adsk.core.ObjectCollection.create()
            inputEntites.add(body1)

            # Get Z axis for circular pattern
            zAxis = rotor.zConstructionAxis

            # Create the input for circular pattern
            circularFeats = rotor.features.circularPatternFeatures
            circularFeatInput = circularFeats.createInput(inputEntites, zAxis)

            # Set the quantity of the elements
            circularFeatInput.quantity = adsk.core.ValueInput.createByReal(numberOfRollers-1)

            # Set the angle of the circular pattern
            circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')

            # Set symmetry of the circular pattern
            circularFeatInput.isSymmetric = True

            # Create the circular pattern
            circularFeat = circularFeats.add(circularFeatInput)

            ToolBodies = adsk.core.ObjectCollection.create()
            for b in circularFeat.bodies:
                if b != body1:
                    ToolBodies.add(b)

            combineInput = rotor.features.combineFeatures.createInput(body1, ToolBodies)
            combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
            combineInput.isNewComponent = False

            rotor.features.combineFeatures.add(combineInput)

            # Cut a hole through the rotor at its own rotational centre (rotor-local
            # origin) for the input bearing's outer race.
            rotorHoleSketch = rotor.sketches.add(root.xYConstructionPlane)
            rotorHoleSketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), bearingODRadius)

            rotorHoleProfile = rotorHoleSketch.profiles.item(0)
            rotorHoleDistance = adsk.core.ValueInput.createByReal(rotorThickness)
            rotor.features.extrudeFeatures.addSimple(rotorHoleProfile, rotorHoleDistance, adsk.fusion.FeatureOperations.CutFeatureOperation)

            # Cut the output holes through the rotor. These sit on a simple circle of
            # radius outputPinCircleRadius around the rotor's OWN rotational centre
            # (rotor-local origin) -- the same centre as the input hole -- so they stay
            # concentric with it after the transform below, exactly like the lobes and
            # the input hole already are. The offsetting needed for the sliding output
            # mechanism to work comes entirely from the *output pins* being fixed to
            # the main axis instead (see the output_bearings component below); the
            # hole positions themselves need no eccentricity term at all -- a pin at
            # local position P (in the fixed output flange's own frame) and a hole at
            # the same local position P (in the rotor's own frame) is what keeps the
            # pin exactly ecc from its hole's centre at every rotation angle, not just
            # this snapshot.
            #
            # The pattern is started half a step (pi/N) away from angle 0 rather than
            # exactly on it, purely to keep the outer edge clearance as generous as
            # possible relative to the (non-circular) lobe boundary -- see
            # minEdgeWallThickness below.
            outputHolePhaseOffset = math.pi / numOutputHoles.value
            outputHoleSketch = rotor.sketches.add(root.xYConstructionPlane)
            outputHoleCircles = outputHoleSketch.sketchCurves.sketchCircles
            for i in range(numOutputHoles.value):
                outputHoleTheta = (2 * math.pi * i / numOutputHoles.value) + outputHolePhaseOffset
                outputHoleX = outputPinCircleRadius * math.cos(outputHoleTheta)
                outputHoleY = outputPinCircleRadius * math.sin(outputHoleTheta)
                outputHoleCircles.addByCenterRadius(adsk.core.Point3D.create(outputHoleX, outputHoleY, 0), outputHoleRadius)

            outputHoleDistance = adsk.core.ValueInput.createByReal(rotorThickness)
            for outputHoleProfile in outputHoleSketch.profiles:
                rotor.features.extrudeFeatures.addSimple(outputHoleProfile, outputHoleDistance, adsk.fusion.FeatureOperations.CutFeatureOperation)

            #Offset the rotor to make the shaft rotat concentric with origin
            transform = rotorOcc.transform
            transform.translation = adsk.core.Vector3D.create(ecc, 0, 0)
            rotorOcc.transform = transform
            des.snapshots.add()

            housingOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            housing = housingOcc.component
            housing.name = 'housing'

            #add a sketch so rotor clearance is obvious
            sketches = housing.sketches
            rotorClearanceSketch = sketches.add(root.xYConstructionPlane)
            sketchCircles = rotorClearanceSketch.sketchCurves.sketchCircles
            centerPoint = adsk.core.Point3D.create(0, 0, 0)
            sketchCircles.addByCenterRadius(centerPoint, rotorRadius)

            #add rollers
            rollerSketch = sketches.add(root.xYConstructionPlane)
            sketchCircles = rollerSketch.sketchCurves.sketchCircles
            centerPoint = adsk.core.Point3D.create(rotorRadius, 0, 0)
            sketchCircles.addByCenterRadius(centerPoint, rollerRadius)

            rollerProfile = rollerSketch.profiles.item(0)
            distance = adsk.core.ValueInput.createByReal(housingThickness)
            rollerExtrudes = housing.features.extrudeFeatures.addSimple(rollerProfile, distance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            # Get the extrusion body
            roller = rollerExtrudes.bodies.item(0)
            roller.name = "roller"

            inputEntites = adsk.core.ObjectCollection.create()
            inputEntites.add(roller)

            # Create the input for circular pattern
            circularFeats = housing.features.circularPatternFeatures
            zAxis = housing.zConstructionAxis
            circularFeatInput = circularFeats.createInput(inputEntites, zAxis)

            # Set the quantity of the elements
            circularFeatInput.quantity = adsk.core.ValueInput.createByReal(numberOfRollers)

            # Set the angle of the circular pattern
            circularFeatInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')

            # Set symmetry of the circular pattern
            circularFeatInput.isSymmetric = True

            # Create the circular pattern
            circularFeat = circularFeats.add(circularFeatInput)

            # ---- Input eccentric ----
            eccentricOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            eccentric = eccentricOcc.component
            eccentric.name = 'input_eccentric'

            eccentricSketch = eccentric.sketches.add(root.xYConstructionPlane)
            eccentricCircles = eccentricSketch.sketchCurves.sketchCircles
            eccentricCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), shaftRadius)
            eccentricCircles.addByCenterRadius(adsk.core.Point3D.create(ecc, 0, 0), bearingIDRadius)

            # Bore (main axis) and OD (offset by ecc) aren't concentric, but the bore
            # nests fully inside the OD (validated above), so this sketch has exactly
            # two profiles: the bore disc (1 loop) and the eccentric ring (2 loops).
            # Select by loop count -- profile ordering isn't guaranteed.
            eccentricProfile = None
            for prof in eccentricSketch.profiles:
                if prof.profileLoops.count == 2:
                    eccentricProfile = prof
                    break

            eccentricDistance = adsk.core.ValueInput.createByReal(bearingWidthCm)
            eccentricExtrudes = eccentric.features.extrudeFeatures.addSimple(eccentricProfile, eccentricDistance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            eccentricExtrudes.bodies.item(0).name = 'input_eccentric'

            # ---- Input bearing (placeholder solid) ----
            bearingOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            bearing = bearingOcc.component
            bearing.name = 'input_bearing'

            bearingSketch = bearing.sketches.add(root.xYConstructionPlane)
            bearingCenter = adsk.core.Point3D.create(ecc, 0, 0)
            bearingSketch.sketchCurves.sketchCircles.addByCenterRadius(bearingCenter, bearingIDRadius)
            bearingSketch.sketchCurves.sketchCircles.addByCenterRadius(bearingCenter, bearingODRadius)

            # Concentric this time, but the same "ring = 2 loops" selection still applies.
            bearingProfile = None
            for prof in bearingSketch.profiles:
                if prof.profileLoops.count == 2:
                    bearingProfile = prof
                    break

            bearingDistance = adsk.core.ValueInput.createByReal(bearingWidthCm)
            bearingExtrudes = bearing.features.extrudeFeatures.addSimple(bearingProfile, bearingDistance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            bearingExtrudes.bodies.item(0).name = 'input_bearing'

            # ---- Output bearings (placeholder solids) ----
            outputBearingOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            outputBearingComp = outputBearingOcc.component
            outputBearingComp.name = 'output_bearings'

            outputBearingSketch = outputBearingComp.sketches.add(root.xYConstructionPlane)
            # Start at the same phase-offset angle as the rotor's output holes (see the
            # comment where outputHolePhaseOffset is computed) -- the circular pattern
            # below then repeats every 2*pi/N from here, landing each bearing exactly
            # inside its matching hole.
            outputBearingCenter = adsk.core.Point3D.create(
                outputPinCircleRadius * math.cos(outputHolePhaseOffset),
                outputPinCircleRadius * math.sin(outputHolePhaseOffset), 0)
            outputBearingSketch.sketchCurves.sketchCircles.addByCenterRadius(outputBearingCenter, outputShaftRadius)
            outputBearingSketch.sketchCurves.sketchCircles.addByCenterRadius(outputBearingCenter, outputBearingRadius)

            outputBearingProfile = None
            for prof in outputBearingSketch.profiles:
                if prof.profileLoops.count == 2:
                    outputBearingProfile = prof
                    break

            # Height matches the rotor thickness, since these bearings need to span the
            # full depth of the rotor's output holes (no separate width was specified).
            outputBearingDistance = adsk.core.ValueInput.createByReal(rotorThickness)
            outputBearingExtrudes = outputBearingComp.features.extrudeFeatures.addSimple(outputBearingProfile, outputBearingDistance, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
            outputBearingBody = outputBearingExtrudes.bodies.item(0)
            outputBearingBody.name = 'output_bearing'

            # Circular-pattern the one bearing (fixed to the main axis, unlike the
            # rotor -- output pins don't orbit) around to the full output hole count.
            outputBearingEntities = adsk.core.ObjectCollection.create()
            outputBearingEntities.add(outputBearingBody)

            outputBearingCircularFeats = outputBearingComp.features.circularPatternFeatures
            outputBearingZAxis = outputBearingComp.zConstructionAxis
            outputBearingCircularInput = outputBearingCircularFeats.createInput(outputBearingEntities, outputBearingZAxis)
            outputBearingCircularInput.quantity = adsk.core.ValueInput.createByReal(numOutputHoles.value)
            outputBearingCircularInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
            outputBearingCircularInput.isSymmetric = True
            outputBearingCircularFeats.add(outputBearingCircularInput)




        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))





class commandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        print("--- commandDestroyHandler ---")
        sys.stdout.close()
        try:
            eventArgs = adsk.core.CommandEventArgs.cast(args)

            # when the command is done, terminate the script
            # this will release all globals which will remove all event handlers
            adsk.terminate()
        except:
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))




