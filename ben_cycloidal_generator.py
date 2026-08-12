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
    rotorRadius = (rotorDia/10)/2
    NumberOfRollers = reductionRatio + 1
    housing_cir = 2 * rotorRadius * math.pi
    rollerRadius = housing_cir / (4 * NumberOfRollers)
    eccentricity = 0.5 * rollerRadius
    return eccentricity, rollerRadius * 10



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
            
            global reductionRatio, depth, rotorDia,  eccentricity, rollerDia

            # Define the command dialog.

            reductionRatio = inputs.addIntegerSpinnerCommandInput('reductionRatio', 'Reduction Ration (x:1)', 3, 100, 1, 30)
            rotorDia = inputs.addValueInput('rotorDia', 'Diameter of the rotor', 'mm', adsk.core.ValueInput.createByReal(10.0))
            depth = inputs.addValueInput('depth', 'Gear Height', 'mm', adsk.core.ValueInput.createByReal(1.0))
            rollerDia = inputs.addValueInput('rollerDia', 'Roller Diameter', 'mm', adsk.core.ValueInput.createByReal(0.0))
            eccentricity = inputs.addTextBoxCommandInput('eccentricity', 'Eccentricity', '', 1, True)

            # re-calculate the eccentricity and update the UI.
            Ecc, rollerRadius = calcEcc(rotorDia.value, reductionRatio.value)
            eccentricity.text = str(round(Ecc, 3)*10) + 'mm'
            print(f'eccentricity: {eccentricity.text}')

            # calculate the roller radius
            rollerDia.value = rollerRadius * 2

            # Connect to the command related events.
            onExecute = commandExecuteHandler()
            cmd.execute.add(onExecute)
            handlers.append(onExecute)        
            
            onInputChanged = commandInputChangedHandler()
            cmd.inputChanged.add(onInputChanged)
            handlers.append(onInputChanged)     
            
            #onValidateInputs = GearCommandValidateInputsHandler()
            #cmd.validateInputs.add(onValidateInputs)
            #handlers.append(onValidateInputs)

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
            
            global eccentricity, rollerDia, rotorDia

            if changedInput.id == 'rotorDia' or changedInput.id == 'reductionRatio':
                # re-calculate the eccentricity and update the UI.
                Ecc, rollerRadius = calcEcc(rotorDia.value, reductionRatio.value)
                eccentricity.text = str(round(Ecc, 3)*10) + 'mm'
                rollerDia.value = rollerRadius * 2
                print(f'Eccentricity has changed to: {eccentricity.text}')

            if changedInput.id == 'rollerDia':
                # re-calculate the rotor Dia
                housingDia = (rollerDia.value / 2) * (4 * (reductionRatio.value + 1))
                rotorDia.value = (housingDia / math.pi)
                
                

        except:
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
            addOrUpdateParam(params, 'rotorDia', rotorDia.value, 'mm', 'Diameter to the centre of the pins')
            addOrUpdateParam(params, 'rollerDia', rollerDia.value, 'mm', 'Diameter of the roller pins')



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

            root = des.rootComponent

            rotorOcc = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
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




