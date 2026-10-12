# Experimental simulator engine changes

The Blender scene updates and runtime graphics controls require local changes to
the Stonefish engines. This directory preserves the exact patches and upstream
commit IDs that were present when `experimental` was merged.

The simulator CMake configure prepares the selected engine automatically. For
manual preparation, run from the repository root:

```sh
git submodule update --init --recursive
python3 tools/prepare_sim_vendor.py stonefish
# Only for an RTX build:
python3 tools/prepare_sim_vendor.py stonefish_rtx
```

Preparation recognizes an already applied patch. Conflicting existing engine
edits cause an error and are preserved. The optional RTX fork is cloned at the
recorded commit. Its CUDA, OptiX, and DLSS dependencies still need to be available.
An external `AUTONOMY_STONEFISH_PREFIX` must contain an engine built with the same
patch; CMake does not modify that external install.

The upstream Stonefish submodule reports local modifications after preparation.
These changes are stored in the parent branch as patches, so a fresh clone can
reproduce them without unpublished commits in a third-party repository.
