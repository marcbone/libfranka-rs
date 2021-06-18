// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later
use crate::robot::service_types::MoveMotionGeneratorMode;

pub trait MotionGeneratorTrait {
    fn get_motion_generator_mode() -> MoveMotionGeneratorMode;
}
