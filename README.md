# Cycloid Generator

A Fusion 360 Python script that parametrically generates the mechanical
components of a single-stage cycloidal drive gearbox:

- **rotor** — the lobed cycloidal disc
- **housing** — the ring of fixed rollers the rotor meshes against
- **input_eccentric** — the eccentric bushing that adapts the input shaft to the input bearing
- **input_bearing** — a simple placeholder body for the input bearing
- **output_bearings** — simple placeholder bodies for the output bearings, patterned around the output holes cut into the rotor

The script builds each body from scratch inside Fusion, live-validates the
dimensions you enter (so it won't let you generate self-intersecting
geometry), and offers guidance readouts for a few values that are easy to
get wrong by hand (minimum bearing sizes, minimum wall thicknesses, valid
output pin circle range).

## Requirements

- **Autodesk Fusion 360**, desktop version (the browser-only version does
  not support Scripts and Add-ins).
- Windows or macOS.

## Installation

### 1. Get the files

Clone the repository, or download it as a ZIP and extract it:

```
git clone https://github.com/capella-ben/CycloidGenerator.git
```

You should end up with a folder named `CycloidGenerator` containing
`ben_cycloidal_generator.py`.

### 2. Copy it into Fusion 360's Scripts folder

Fusion looks for scripts in a specific folder, one subfolder per script.
Copy the whole `CycloidGenerator` folder into it:

- **Windows:** `%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts\`
- **macOS:** `~/Library/Application Support/Autodesk/Autodesk Fusion 360 API/Scripts/`

If you're not sure where that is on your machine, you can get Fusion to
show you: open the **Scripts and Add-ins** dialog in Fusion (see step 3
below), select the **Scripts** tab, click the green **+** button to add a
script, and the file browser that opens will already be pointed at the
correct folder — copy `CycloidGenerator` in there (or navigate up one
level first if it opens inside an existing script folder), then cancel
out of the add dialog and refresh the list.

> **Note:** Fusion normally expects a script's main `.py` file to share
> its folder's name (e.g. `CycloidGenerator/CycloidGenerator.py`). This
> repo instead uses `CycloidGenerator/ben_cycloidal_generator.py`. If the
> script doesn't show up in the list after copying it into place, try
> renaming `ben_cycloidal_generator.py` to `CycloidGenerator.py` (or
> renaming the folder to `ben_cycloidal_generator` so the names match).

> **Note:** the script currently redirects its debug output to a hardcoded
> path, `D:\temp\python_output.txt` (near the top of
> `ben_cycloidal_generator.py`). On a machine without a `D:` drive (or on
> macOS) this line will throw an error as soon as you run the script.
> Either create that folder, or edit/remove that line before running it.

### 3. Enable and run it in Fusion

1. In Fusion, open the **Utilities** tab of the ribbon and click **Add-Ins**
   (or press `Shift+S`) to open the **Scripts and Add-ins** dialog.
2. Select the **Scripts** tab. `CycloidGenerator` should appear under **My
   Scripts** — select it.
3. Click **Run** to launch it, or **Debug** if you're attaching a debugger
   (see `.vscode/launch.json` — the workspace is already set up for the
   "Python: Attach" workflow used by Autodesk's Fusion 360 VS Code
   extension).
4. Optionally, click **Add to Toolbar** so it's one click away next time.

A dialog will appear with the generator's inputs (see below). Set your
dimensions and click **OK** to generate the components in the active
design.

## Using the generator

The dialog is organized top to bottom as: overall gear sizing, the input
shaft/eccentric/bearing stage, and the output holes/bearing stage. Fields
marked *(read-only)* are calculated guidance, not inputs — they update
live as you change the values they depend on.

| Field | Description |
|---|---|
| Reduction Ration (x:1) | Gearbox reduction ratio. Also sets the number of housing rollers (ratio + 1) and rotor lobes (ratio). |
| Roller Pitch Circle Diameter (PCD) | Diameter of the circle the housing rollers are centred on. |
| Gear Height | Extrusion height (thickness) of the rotor and housing. |
| Roller Diameter | Diameter of each housing roller. Auto-calculated from the PCD and reduction ratio, but editable (editing it back-solves the PCD instead). |
| Eccentricity *(read-only)* | The eccentric offset between the rotor's own centre and the main axis, derived from the PCD and reduction ratio. |
| Minimum Wall Thickness | Minimum wall thickness used for the bearing-sizing guidance below (both input and output sides). |
| Input Shaft Diameter | Diameter of the shaft driving the input eccentric. |
| Min. Recommended Bearing Bore *(read-only)* | Smallest input bearing bore (ID) that leaves at least Minimum Wall Thickness of material in the eccentric bushing. |
| Bearing Bore (ID) | Bore of the input bearing. Also the OD of the `input_eccentric` body. |
| Bearing Outer Diameter (OD) | OD of the input bearing. Also the diameter of the hole cut into the rotor for it. |
| Bearing Width | Width (extrusion height) of the input eccentric and input bearing. |
| Number of Output Holes | Number of output holes/bearings (minimum 3). |
| Output Shaft Diameter | Diameter of each output shaft/pin. Also the bore of each output bearing (nominal fit). |
| Min. Recommended Output Bearing OD *(read-only)* | Smallest output bearing OD that leaves at least Minimum Wall Thickness of material around the output shaft bore. |
| Output Bearing Diameter (OD) | OD of each output bearing. |
| Minimum Wall Thickness (Holes to Outer Edge) | Minimum material required between any hole cut into the rotor (input or output) and the rotor's outer (lobe) boundary. Enforced as a hard limit. |
| Output Pin Circle Diameter Range *(read-only)* | Valid min–max range for the field below, given the current dimensions. |
| Output Pin Circle Diameter | Diameter of the circle the output holes/bearings are centred on (concentric with the input hole). |
| Validation Status *(read-only)* | Explains why **OK** is disabled, if the current dimensions can't be built (e.g. holes overlapping, or too close to an edge). |

If the current combination of dimensions can't produce valid geometry, the
**OK** button disables itself and **Validation Status** explains why —
adjust values until it clears rather than needing to re-run the script.

## Notes

- Re-running the script on a document from a previous run is safe — it
  updates the existing user parameters and geometry in place rather than
  erroring.
- The generated bearing bodies (`input_bearing`, `output_bearings`) are
  simple placeholder rings (ID → OD, extruded to width), not detailed
  models of real bearings — they're there to verify fit/clearance in the
  assembly, not for manufacturing.
- No output-side housing/end-plate geometry is generated yet, and no
  physical shaft bodies are modelled — only the hole/eccentric/bearing
  geometry those parts need to mate with.

## Credits

Original cycloidal profile generator by mawildoer.
