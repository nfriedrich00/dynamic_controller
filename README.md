# Dynamic controller

This nav2 controller plugin allows to dynamically switch between multiple controllers while moving.

## Description
The controller will switch every 30 seconds between the rpp and mppi controller. When using the example config, you can distinguish between the controllers by the difference in speed.

## Getting Started

### Dependencies

* Nav2

### Installing

Clone and build. To use the plugin as controller plugin, you need to source the package folder.

### Executing program

Load this controller as nav2 controller (see example config). 

## Authors

[Nils Friedrich](mailto:nils-jonathan.friedrich@student.tu-freiberg.de)

## Version History

* 0.1
    * Initial Release