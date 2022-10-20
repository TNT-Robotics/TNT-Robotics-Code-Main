
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
**Bold** - 

## Basic move methods

#### GoForward (double power, int time)
Moves the bot forward for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of backward driving. Additionally, a [telemetry](#telemetry) update is called. 

| Action   | Type    |
| :---------| :------- |
| Input    | Double & Integer  |
| Output   | Void |
<details>
  <summary>Function code</summary>
  
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


#### GoBackward (double power, int time)
Moves the bot backward for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of forward driving. Additionally, a [telemetry](#telemetry) update is called. 

| Action   | Type    |
| :---------| :------- |
| Input    | Double & Integer  |
| Output   | Void |
<details>
  <summary>Function code</summary>
  
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

#### dontMove (*int time*)
Additionally, a [telemetry](#telemetry) update is called. 

| Action   | Type    |
| :---------| :------- |
| Input    | Double & Integer  |
| Output   | Void |
<details>
  <summary>Function code</summary>
  
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

