/**
 * Autonomous programs:
 * AutonomousTemplate - The base autonomous class that all others extend
 * DuckPark  - Duck side -> park in storage (duck side emergency)
 * DuckStorage - Duck side -> Preload -> duck -> park in storage (duck side backup)
 * DuckWarehouse - Duck side -> Preload -> duck -> park in warehouse (duck sde main)
 * WarehousePark - Warehouse side -> park in warehouse (warehouse side emergency)
 * WarehousePreload - Warehouse side -> wait 15 seconds -> Preload -> park in warehouse (warehouse side backup)
 * Warehouse - Warehouse side -> Preload -> 3 intake cycles -> park in warehouse (warehouse side main)
 * SeaLion - Warehouse side -> Preload -> 2 intake cycles -> park in warehouse (warehouse side, made to work well with the Sea Lion Techs)
 * Remote - basically the same as RedDuckWarehouse (for remote competitions); no longer maintained
 */
package org.firstinspires.ftc.teamcode.opmodes.autonomous;