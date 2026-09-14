"""Official source models retained for the project's tabletop manipulation tasks."""

# Paths relative to the versioned asset root's Isaac/Props directory.
# Keep selection shared by the downloader and gallery; source dependencies are
# discovered from USD rather than maintained as a second file list.
ISAACSIM_PROPS = {
    "objects": (
        "YCB/Axis_Aligned_Physics/004_sugar_box.usd",
        "YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd",
        "YCB/Axis_Aligned/007_tuna_fish_can.usd",
        "YCB/Axis_Aligned/008_pudding_box.usd",
        "YCB/Axis_Aligned/009_gelatin_box.usd",
        "YCB/Axis_Aligned/010_potted_meat_can.usd",
        "YCB/Axis_Aligned/061_foam_brick.usd",
        "Food/mac_n_cheese_centered.usd",
        "Blocks/blue_block.usd",
        "Blocks/green_block.usd",
        "Blocks/red_block.usd",
        "Blocks/yellow_block.usd",
    ),
    "containers": (
        "YCB/Axis_Aligned/024_bowl.usd",
        "KLT_Bin/small_KLT.usd",
    ),
    "workspaces": ("Mounts/ThorlabsTable/table_instanceable.usd",),
}
