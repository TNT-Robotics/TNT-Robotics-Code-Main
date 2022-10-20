
# TNT Robotics Code Documentation
#### Written by [Tomas Pokorny](  mailto:tomasekpokorny02@gmail.com?subject=TNT%20Robotics%20Code)
##### Last updated - 19/10/2022

## [! Original ReadMe with FTC Notes !](https://github.com/TNT-Robotics/TNT-Robotics-Code-Main/blob/main/FTCReadMe.md)

## Table of Contents
- [Definitions / Rules](#Definitions/Rules)
- [Basic move methods](#Basic%20move%20methods)
- [Complex move methods](#example2)
- [Arm methods](#third-example)
- [Vision methods](#fourth-examplehttpwwwfourthexamplecom)
- [Misc methods](#misc)
- [Configuration file](#config)

## Definitions / Rules

*Italics* - Overloaded variable <br>
**Bold** - Important variable / concept

## Basic move methods

#### goForward (double power, int time)
Moves the bot forward for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of backward driving. 

Additionally, a [telemetry](#telemetry) update is called. 

| Action   | Type    |
| :---------| :------- |
| Input    | Double & Integer  |
| Output   | Void |
<details>
  <summary>Method code</summary>
  
  ```java
  public void goForward(double power, int time) {
        newFunctions.updateTele("Going forward with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(power);
        newFunctions.slp(time);
        dontMove();
    }
  ```
</details>


#### goBackward (double power, int time)
Moves the bot backward for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of forward driving. 

Additionally, a [telemetry](#telemetry) update is called. 

| Action   | Type    |
| :---------| :------- |
| Input    | Double & Integer  |
| Output   | Void |
<details>
  <summary>Method code</summary>
  
  ```java
  public void goBackward(double power, int time) {
        power *= -1;

        newFunctions.updateTele("Going backwards with power " + power + " for " + time + "ms", 0);

        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(power);
        newFunctions.slp(time);
        dontMove();
    }
  ```
</details>

#### diagonalLeft (double power, int time)
Moves the bot diagonally left for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of diagonal-right driving. 

Additionally, a [telemetry](#telemetry) update is called. 

| Action   | Type    |
| :---------| :------- |
| Input    | Double & Integer  |
| Output   | Void |
<details>
  <summary>Method code</summary>
  
  ```java
  public void diagonalLeft(double power, int time) {
        newFunctions.updateTele("Strafing diagonal left with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(0);
        newFunctions.slp(time);
        dontMove();
    }
  ```
</details>

#### diagonalRight (double power, int time)
Moves the bot diagonally right for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of diagonal-left driving. 

Additionally, a [telemetry](#telemetry) update is called. 

| Action   | Type    |
| :---------| :------- |
| Input    | Double & Integer  |
| Output   | Void |
<details>
  <summary>Method code</summary>
  
  ```java
  public void diagonalRight(double power, int time) {
        newFunctions.updateTele("Strafing diagonal right with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(power);
        newFunctions.slp(time);
        dontMove();
    }
  ```
</details>

#### dontMove (*int time*)
A function that turns off all 4 of the robots drive motors. *Time* variable may be used to overload a wait functionality to the method.

Additionally, a [telemetry](#telemetry) update is called. 

| Action   | Type    |
| :---------| :------- |
| Input    | *Integer*  |
| Output   | Void |
<details>
  <summary>Method code</summary>
  
  ```java
  public void dontMove() {
        newFunctions.updateTele("Stopped", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(0);
    }
  ```
</details>
<details>
  <summary>Overloaded method code</summary>
  
  ```java
  public void dontMove(int time) {

        newFunctions.updateTele("Waiting for " + time + "ms", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(0);
        newFunctions.slp(time);
    }
  ```
</details>



