// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains model VirtualWallCuboid type.
use crate::robot::service_types::GetCartesianLimitResponse;

///Parameters of a cuboid used as virtual wall.
pub struct VirtualWallCuboid {
    ///ID of the virtual wall.
    pub id: i32,
    ///Corner point of the cuboid in world frame in \[m\].
    pub object_world_size: [f64; 3],
    ///4x4 transformation matrix, column-major.
    pub p_frame: [f64; 16],
    ///True if this Cartesian limit is active, false otherwise.
    pub active: bool,
}

impl VirtualWallCuboid {
    pub fn new(id: i32, response: GetCartesianLimitResponse) -> Self {
        VirtualWallCuboid {
            id,
            object_world_size: response.object_world_size,
            p_frame: response.object_frame,
            active: response.object_activation,
        }
    }
}
