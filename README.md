
# TNT Robotics Code Documentation
#### Written by [Tomas Pokorny](  mailto:tomasekpokorny02@gmail.com?subject=TNT%20Robotics%20Code)
##### Last updated - 19/10/2022

## [! Original ReadMe with FTC Notes !](https://github.com/TNT-Robotics/TNT-Robotics-Code-Main/blob/main/FTCReadMe.md)

### Completion of Documentation
- [x] Definitions / Rules
- [x] Basic move methods
- [ ] Complex move methods
- [ ] Arm methods
- [ ] Vision methods
- [ ] Misc methods
- [ ] Configuration file

## Table of Contents
- [Definitions / Rules](#Definitions-/-Rules)
- [Basic move methods](#Basic-move-methods)
- [Complex move methods](#example2)
- [Arm methods](#third-example)
- [Vision methods](#fourth-examplehttpwwwfourthexamplecom)
- [Miscellaneous methods](#Miscellaneous)
- [Configuration file](#config)

## Definitions / Rules

*Italics* - Overloaded variable <br>
**Bold** - Important variable / concept

## Basic move methods
### Methods that are used to control the movement of the robot. They allow movement in any direction.

#### goForward (double power, int time)
Moves the bot forward for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of backward driving. 

Additionally, a [telemetry](#updateTele) update is called. 

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

---

#### goBackward (double power, int time)
Moves the bot backward for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of forward driving. 

Additionally, a [telemetry](#updateTele) update is called. 

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

---

#### diagonalLeft (double power, int time)
Moves the bot diagonally left for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of diagonal-right driving. 

Additionally, a [telemetry](#updateTele) update is called. 

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

---

#### diagonalRight (double power, int time)
Moves the bot diagonally right for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of diagonal-left driving. 

Additionally, a [telemetry](#updateTele) update is called. 

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

---

#### turnLeft (double power, int time)
Turn the bot left for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of weird backward turning, **definitely not recommended.**

Additionally, a [telemetry](#updateTele) update is called. 

| Action   | Type    |
| :---------| :------- |
| Input    | Double & Integer  |
| Output   | Void |
<details>
  <summary>Method code</summary>
  
  ```java
  public void turnLeft(double power, int time) {
        newFunctions.updateTele("Turning left with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(power);
        newFunctions.slp(time);
        dontMove();
    }
  ```
</details>

---

#### turnRight (double power, int time)
Turn the bot right for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of weird backward turning, **definitely not recommended.**

Additionally, a [telemetry](#updateTele) update is called. 

| Action   | Type    |
| :---------| :------- |
| Input    | Double & Integer  |
| Output   | Void |
<details>
  <summary>Method code</summary>
  
  ```java
  public void turnRight(double power, int time) {
        newFunctions.updateTele("Turning right with power " + power + " for " + time + "ms", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(0);
        newFunctions.slp(time);
        dontMove();
    }
  ```
</details>

---

#### turn90Left ()
Turn the bot 90 degrees to the left. This is accomplished by turning left with maximum power for 500ms.

Additionally, a [telemetry](#updateTele) update is called. 

| Action   | Type    |
| :---------| :------|
| Input    | Void    |
| Output   | Void    |
<details>
  <summary>Method code</summary>
  
  ```java
  public void turn90left( ) {
        newFunctions.updateTele("Doing 90 degrees left turn!", 0);
        cfg.getRfD().setPower(1);
        cfg.getLfD().setPower(0);
        cfg.getLbD().setPower(0);
        cfg.getRbD().setPower(1);
        newFunctions.slp(500);
        dontMove();
    }
  ```
</details>

---

#### turn90Right ( )
Turn the bot 90 degrees to the right. This is accomplished by turning right with maximum power for 500ms.

Additionally, a [telemetry](#updateTele) update is called. 

| Action   | Type    |
| :---------| :------|
| Input    | Void    |
| Output   | Void    |
<details>
  <summary>Method code</summary>
  
  ```java
  public void turn90right() {
        newFunctions.updateTele("Doing 90 degrees right turn!", 0);
        cfg.getRfD().setPower(0);
        cfg.getLfD().setPower(1);
        cfg.getLbD().setPower(1);
        cfg.getRbD().setPower(0);
        newFunctions.slp(500);
        dontMove();
    }
  ```
</details>

---

#### strafeLeft (double power, int time)
The bot strafes to the left for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of possibly strafing right. **(not tested)**

Additionally, a [telemetry](#updateTele) update is called. 

| Action   | Type    |
| :---------| :------- |
| Input    | Double & Integer  |
| Output   | Void |
<details>
  <summary>Method code</summary>
  
  ```java
  public void strafeLeft(double power, int time) {
        newFunctions.updateTele("Strafing left with power " + power + " for " + time + " ms.", 0);
        cfg.getRfD().setPower(power);
        cfg.getLfD().setPower(power * -1);
        cfg.getLbD().setPower(power);
        cfg.getRbD().setPower(power * - 1);
        newFunctions.slp(time);
        dontMove();
    }
  ```
</details>

---

#### strafeRight (double power, int time)
The bot strafes to the right for the given **time** in milliseconds and given **power** from 0,1. Value from 0, -1 for power is also permitted with the effect of possibly strafing left. **(not tested)**

Additionally, a [telemetry](#updateTele) update is called. 

| Action   | Type    |
| :---------| :------- |
| Input    | Double & Integer  |
| Output   | Void |
<details>
  <summary>Method code</summary>
  
  ```java
  public void strafeRight(double power, int time) {
        newFunctions.updateTele("Strafing left with power " + power + " for " + time + " ms.", 0);
        cfg.getRfD().setPower(power * -1);
        cfg.getLfD().setPower(power);
        cfg.getLbD().setPower(power * -1);
        cfg.getRbD().setPower(power);
        newFunctions.slp(time);
        dontMove();
    }
  ```
</details>

---


#### dontMove (*int time*)
A function that turns off all 4 of the robots drive motors. *Time* argument may be used to overload a wait functionality to the method.

Additionally, a [telemetry](#updateTele) update is called. 

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

## Miscellaneous
### Methods that are usually helper methods for larger methods
---

<h4 id="updateTele">updateTele (String action, int statusNum)</h4>
Updates the telemetry on the driver's hub. The argument of **action** is setting the display name of what is written on the screen while the **status argument** is a simple form of telling the driver that something either works, is in danger or is not working. </br> </br>
Status values meanings -> 
0 - Nominal, everything is working as planned 
1 - Warning, write down and remember to fix after competition  
2 - Minor error, consider fixing before next match if appliciable 
3 - Fatal error, robot is not performing as expected, possibility of turning off should be considered.

| Action   | Type    |
| :---------| :------- |
| Input    | String & Integer  |
| Output   | Void (Driver's hub) |
<details>
  <summary>Method code</summary>
  
  ```java
  public void updateTele(String action, int statusNum) {
        telemetry.addData("Status", statusNum);
        telemetry.addData("Action", action);
        telemetry.addData("Running for", cfg.getrTime().toString());
        telemetry.update();
    }
  ```
</details>




