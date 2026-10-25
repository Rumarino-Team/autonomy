#pragma once

#include <stdint.h>

/* Stonefish physics mesh for the Hydrus compound, dumped from hydrus_auv.scn.
   Vertices are in the center-of-mass frame. origin_basis is row-major and maps
   part-origin vectors into that frame. */

typedef struct {
    float vertex[9];
    uint32_t part;
} AuvDragFace;

typedef struct {
    double cd[3];
    double cf[3];
    double origin_basis[9];
} AuvDragPart;

extern const AuvDragPart hydrus_drag_parts[];
extern const uint32_t hydrus_drag_part_count;
extern const AuvDragFace hydrus_drag_faces[];
extern const uint32_t hydrus_drag_face_count;
