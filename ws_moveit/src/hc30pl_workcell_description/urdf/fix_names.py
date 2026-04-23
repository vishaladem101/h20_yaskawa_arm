import os

for f in ["hc30pl.xacro", "hc30pl_macro.xacro"]:
    with open(f, "r") as file:
        txt = file.read()
    txt = txt.replace("motoman_hc20_support", "hc30pl_workcell_description")
    txt = txt.replace("meshes/hc20", "meshes/hc30pl")
    txt = txt.replace("hc20", "hc30pl")
    with open(f, "w") as file:
        file.write(txt)
