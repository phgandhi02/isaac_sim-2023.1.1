# Changelog

## [0.7.0] - 2023-11-29
### Changed 
- Location on the Menu reverted back to from Create->Isaac->End Effector

## [0.6.0] - 2023-11-20
### Changed 
- Location on the Menu changed from Create->Isaac->End Effector to Create->Isaac->Robot.

## [0.5.0] - 2023-09-26
### Changed 
- Updated ogn node to prevent issue where it would only call for close on first attempt.
- Automatically move gripper origin out of collision body, with a warning to tell how much to push it forward 
## [0.4.1] - 2023-08-11
### Changed 
- Updated ogn node prims types from bundle to ogn

## [0.4.0] - 2023-01-03
### Changed
- Update to new action based menu system

## [0.3.2] - 2022-12-01
### Fixed
- CreateSurfaceGripper command documentation update


## [0.3.1] - 2022-12-01
### Fixed
- CreateSurfaceGripper command .do() only returns the created prim and not a tuple

## [0.3.0] - 2022-10-27
### Added
- Moved surface gripper to Create -> Isaac -> End Effector menu
- More Robots, Environments to menu

## [0.2.0] - 2022-10-14
### Added
- Logic for keeping surface gripper attempting to close until it reaches some object

## [0.1.3] - 2022-09-07
### Fixed
- Fixes for kit 103.5

## [0.1.2] - 2022-05-10

### Changed
- Change Tests to use meters as distance unit

## [0.1.1] - 2022-05-03

### Added
- Surface Gripper Omnigraph Node

## [0.1.0] - 2021-07-31

### Added
- Initial version of extension
