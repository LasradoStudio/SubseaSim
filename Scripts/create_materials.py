"""Creates the plugin's one material asset from code, so the project needs no hand-authored content.

M_SubseaSurface: BaseColor = Color x VertexColor x (procedural world-space detail, faded by
DetailStrength); Roughness and Metallic are parameters. Every surface in the simulation is a
dynamic instance of it.

Run headless (the build script does this when the asset is missing):
    UnrealEditor-Cmd.exe SubseaSim.uproject -run=pythonscript -script=Scripts/create_materials.py -unattended
"""

import unreal

PATH = "/SubseaSim/Materials"
NAME = "M_SubseaSurface"

mel = unreal.MaterialEditingLibrary
assets = unreal.EditorAssetLibrary
full = f"{PATH}/{NAME}"

if assets.does_asset_exist(full):
    assets.delete_asset(full)

material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(NAME, PATH, unreal.Material, unreal.MaterialFactoryNew())


def node(cls, x, y, **props):
    expression = mel.create_material_expression(material, cls, x, y)
    for key, value in props.items():
        expression.set_editor_property(key, value)
    return expression


def scalar(name, default, x, y):
    return node(unreal.MaterialExpressionScalarParameter, x, y, parameter_name=name, default_value=default)


color = node(unreal.MaterialExpressionVectorParameter, -900, -300, parameter_name="Color", default_value=unreal.LinearColor(0.5, 0.5, 0.5, 1.0))
vertex = node(unreal.MaterialExpressionVertexColor, -900, -120)
tinted = node(unreal.MaterialExpressionMultiply, -600, -220)
mel.connect_material_expressions(color, "", tinted, "A")
mel.connect_material_expressions(vertex, "", tinted, "B")

# Multi-octave noise in world space: sand grain, marine growth and stains that stay put as the camera moves.
world = node(unreal.MaterialExpressionWorldPosition, -1100, 120)
noise = node(unreal.MaterialExpressionNoise, -850, 120, scale=0.025, levels=5, output_min=0.5, output_max=1.18)
mel.connect_material_expressions(world, "", noise, "Position")
one = node(unreal.MaterialExpressionConstant, -850, 0, r=1.0)
detail_strength = scalar("DetailStrength", 0.0, -850, 300)
detail = node(unreal.MaterialExpressionLinearInterpolate, -600, 60)
mel.connect_material_expressions(one, "", detail, "A")
mel.connect_material_expressions(noise, "", detail, "B")
mel.connect_material_expressions(detail_strength, "", detail, "Alpha")

base = node(unreal.MaterialExpressionMultiply, -350, -100)
mel.connect_material_expressions(tinted, "", base, "A")
mel.connect_material_expressions(detail, "", base, "B")
mel.connect_material_property(base, "", unreal.MaterialProperty.MP_BASE_COLOR)

mel.connect_material_property(scalar("Roughness", 0.8, -350, 150), "", unreal.MaterialProperty.MP_ROUGHNESS)
mel.connect_material_property(scalar("Metallic", 0.0, -350, 260), "", unreal.MaterialProperty.MP_METALLIC)

material.set_editor_property("used_with_instanced_static_meshes", True)
mel.recompile_material(material)
assets.save_asset(material.get_path_name(), only_if_is_dirty=False)
unreal.log(f"SubseaSim: created {material.get_path_name()}")
