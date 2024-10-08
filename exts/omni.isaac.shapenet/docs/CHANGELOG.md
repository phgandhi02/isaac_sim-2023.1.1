# Changelog

## [2.1.1] - 2023-10-04
### Added
- Deprecation warning to UI and shapenet_convert

## [2.1.0] - 2023-09-06
### Fixed
- Remove explicit certifi dependency and use omni.kit.pip_archive

## [2.0.3] - 2023-08-09
### Fixed
- Switch from ui.Window to ScrollingWindow wrapper for extension because scrolling was broken

## [2.0.2] - 2023-07-05
### Changed
- Added explicit omni.kit.pipapi dependency

## [2.0.1] - 2023-01-06
### Fixed
- onclick_fn warning when creating UI

## [2.0.0] - 2022-07-29

### Removed
- web access to models
- http server for receiving external commands

## [1.0.2] - 2022-05-24

### Fixed
- Make webserver thread a daemon so it does not block fast exit

## [1.0.1] - 2022-01-07

### Changed
- Add defaults to http commands
- clearer message when shapenet.org login fails
- allow random ids to work outside the ui

### Fixed
- jupyter_notebook.sh can now take file names with spaces in them

## [1.0.0] - 2021-08-13

### Added
- Initial version of Isaac Sim ShapeNet Extension
