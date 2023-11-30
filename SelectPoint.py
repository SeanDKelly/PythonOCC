from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_PickPoint
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Display.SimpleGui import init_display

# Create a shape (replace this with your actual shape creation)
# For example, use BRepBuilderAPI_MakeBox to create a box
# shape = ...

# Initialize the display
display, start_display, add_menu, add_function_to_menu = init_display()

# Display the shape
display.DisplayShape(shape)

# Allow the user to pick a point on a face
pick_point = BRepBuilderAPI_PickPoint(display)

# Wait for user interaction
start_display()

# Get the selected point and face
selected_point = pick_point.Picked()
selected_face = pick_point.Face()

# Print the selected point coordinates
print("Selected Point:", selected_point.X(), selected_point.Y(), selected_point.Z())

# Print information about the selected face
if not selected_face.IsNull():
    print("Selected Face:", selected_face)
else:
    print("No face selected.")

# Optionally, you can perform further operations with the selected point or face
