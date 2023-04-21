use crate::gripper::types::CommandHeader;

pub trait DeviceData {
    type CommandHeader: CommandHeader;
    type CommandEnum;
    fn create_header(
        command_id: &mut u32,
        command: Self::CommandEnum,
        size: usize,
    ) -> Self::CommandHeader;
    fn get_library_version() -> u16;
}
